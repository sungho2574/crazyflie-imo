"""강화학습 레이싱 정책 실기체 비행 — 상태를 무선으로 받아 정책이 각속도·추력을 보낸다.

    ros2 launch crazyflie_rl launch.py                          # 서버(모캡·PID·rate 모드·로깅)
    ros2 run crazyflie_rl rl_flight --dry-run                   # 정책 로드·첫 행동만 확인
    ros2 run crazyflie_rl rl_flight --shadow                    # 관측만: 정책 출력을 기록 (명령 안 보냄)
    ros2 run crazyflie_rl rl_flight                             # 실제 비행

루프 (100 Hz, 학습 control_hz):
    무선 로그(rl_pv · rl_att · rl_gyro) → 관측 49 → 정책 → [ωx, ωy, ωz, 총추력]
    → cmd_vel_legacy (RPYT rate 모드: 각속도 deg/s + 기준 추력 PWM) → 펌웨어 PID 각속도 루프

순서
    1. 지상에서 추력 0 레거시 패킷으로 thrust lock 해제 → HLC 로 이륙, start 상공 정렬
    2. 호버 중 모터 PWM 평균으로 추력 매핑 보정 (학습 호버 추력 ↔ 실제 호버 PWM)
    3. 정책 루프. 게이트 7 통과 후 결승점에 0.5 s 머무르면 성공
    4. 스트리밍 중단(notifySetpointsStop) → HLC 로 현재 위치 유지 → 착륙
    안전 중단(기울기·방 경계·고도·데이터 끊김·시간 초과) 도 4 로 넘긴다.

⚠️ 전제 (launch.py 가 설정하는 것들)
    - 모캡(Kalman) 상태추정. 정책은 2.6 m/s 까지 나므로 Flow deck 추정으로는 안 된다.
    - stabilizer.controller = 1 (PID). Mellinger 는 레거시 rate 명령을 받지 않고 수평을 잡으려 한다.
    - flightmode.stabModeRoll/Pitch = 0 (rate). yaw 는 기본이 rate.
    - 조종기 emergency 를 항상 손 닿는 곳에. 첫 비행은 반드시 --shadow 로 관측부터 확인할 것.
"""
import argparse
import csv
import math
import os
import time

import numpy as np

from crazyflie_racing import gate_course as gc

from .policy import RacingPolicy
from .racing_obs import RacingObserver, quat_to_matrix

SIM_MASS = 0.0319                  # 학습 기체(crazyflow cf2x_L250) 질량 — 호버 추력 기준
G = 9.81
PWM_FULL = 65535.0
DEFAULT_THRUST_MAX = 0.12          # crazyflow 모델: 모터 추력 = PWM/65535 × 0.12 N (보정 전 기본)
PWM_MIN, PWM_MAX = 1001, 60000     # 레거시 명령: 1000 미만은 0 으로 처리, 60000 상한


def parse_args():
    p = argparse.ArgumentParser(description='강화학습 레이싱 정책 실기체 비행')
    p.add_argument('--model-dir', default=None, help='정책 폴더 (기본: 패키지 models/)')
    p.add_argument('--gates', default=None, help='gates.yaml (기본: crazyflie_racing config)')
    p.add_argument('--shadow', action='store_true',
                   help='명령을 보내지 않고 관측·정책 출력만 기록 (다른 방법으로 비행 중일 때)')
    p.add_argument('--dry-run', action='store_true', help='ROS 없이 정책 로드·첫 행동만 확인')
    p.add_argument('--hover-pwm', type=float, default=None,
                   help='호버 PWM 을 직접 지정 (기본: 이륙 후 호버 중 측정)')
    p.add_argument('--calib-time', type=float, default=2.0, help='호버 PWM 측정 시간 [s]')
    p.add_argument('--max-time', type=float, default=25.0, help='정책 비행 최대 시간 [s]')
    p.add_argument('--max-tilt', type=float, default=70.0, help='이 기울기 넘으면 중단 [deg]')
    p.add_argument('--wall-margin', type=float, default=0.3, help='방 벽까지 이보다 가까우면 중단 [m]')
    p.add_argument('--min-z', type=float, default=0.2, help='이보다 낮으면 중단 [m]')
    p.add_argument('--stale', type=float, default=0.1, help='상태 로그가 이만큼 끊기면 중단 [s]')
    p.add_argument('--aperture-margin', type=float, default=0.0,
                   help='게이트 통과 판정 개구부 여유 [m] (학습 기준 0 = 반폭 0.19 m)')
    p.add_argument('--log-dir', default='~/flight_logs', help='매 스텝 CSV 기록 폴더')
    p.add_argument('--record', action='store_true', help='rosbag 도 기록')
    args, _ = p.parse_known_args()
    return args


class StateBuffer:
    """무선 로그 3개 블록의 최신값과 수신 시각."""

    def __init__(self):
        self.pv = self.att = self.gyro = None
        self.t_pv = self.t_att = self.t_gyro = -1.0

    def on_pv(self, msg, now):
        self.pv, self.t_pv = np.array(msg.values, dtype=float), now

    def on_att(self, msg, now):
        self.att, self.t_att = np.array(msg.values, dtype=float), now

    def on_gyro(self, msg, now):
        self.gyro, self.t_gyro = np.array(msg.values, dtype=float), now

    def age(self, now):
        return now - min(self.t_pv, self.t_att, self.t_gyro)

    def ready(self):
        return self.pv is not None and self.att is not None and self.gyro is not None

    def state(self):
        """(pos, vel, quat_xyzw, omega[rad/s], motor_pwm[4])."""
        return (self.pv[:3], self.pv[3:6], self.att[:4],
                np.radians(self.gyro[:3]), self.att[4:8])


class ThrustMap:
    """학습 단위(N) ↔ 실기체 PWM. 호버 PWM ↔ 학습 기체 호버 추력으로 맞춘다."""

    def __init__(self, hover_pwm=None):
        if hover_pwm is None:                         # 보정 전: crazyflow 선형 모델
            self.pwm_per_newton = PWM_FULL / DEFAULT_THRUST_MAX
        else:
            self.pwm_per_newton = hover_pwm / (SIM_MASS * G / 4)

    def motor_force(self, pwm):
        return np.asarray(pwm, dtype=float) / self.pwm_per_newton

    def base_pwm(self, total_thrust):
        return int(np.clip(total_thrust / 4 * self.pwm_per_newton, PWM_MIN, PWM_MAX))


def to_legacy(cmd, thrust_map):
    """[ωx, ωy, ωz (rad/s), T (N)] → cmd_vel_legacy Twist 값.

    crazyflie_server: roll = linear.y, pitch = -linear.x, yawrate = angular.z, thrust = linear.z
    펌웨어 RPYT rate 모드 + PID (controller_pid.c):
        roll  : attitudeRate.roll  = roll         ↔ gyro.x      → linear.y  = +ωx
        pitch : attitudeRate.pitch = pitch        ↔ -gyro.y     → linear.x  = +ωy
        yaw   : attitudeRate.yaw   = -yawrate     ↔ yaw(CCW)    → angular.z = -ωz
    (단위 deg/s. 추력은 모터 기준 PWM — 파워 분배가 여기에 각 축 보정을 더한다.)
    """
    wx, wy, wz = np.degrees(cmd[:3])
    # 반환 순서 = send(roll→linear.y, pitch→linear.x, yawrate→angular.z, thrust→linear.z)
    return float(wx), float(wy), float(-wz), float(thrust_map.base_pwm(cmd[3]))


def dry_run(policy, observer, finish):
    """start 호버 상태의 관측으로 첫 행동을 계산해 본다 (ROS 불필요)."""
    hover_force = np.full(4, SIM_MASS * G / 4)
    obs = observer.observe(finish, np.zeros(3), [0, 0, 0, 1], np.zeros(3), hover_force, np.zeros(4))
    cmd = policy.command(policy.act(obs))
    print(f'  start 호버 관측 → 첫 명령: ω = {np.round(cmd[:3], 3)} rad/s, '
          f'총추력 {cmd[3]:.3f} N (학습 호버 {SIM_MASS * G:.3f} N)')


def main():
    args = parse_args()
    policy = RacingPolicy(args.model_dir)
    course = gc.load_course(args.gates)
    finish = course.start_hover()
    observer = RacingObserver(course, finish, args.aperture_margin)
    print(f'[rl_flight] 정책 로드 (iteration {policy.iteration}, {policy.control_hz} Hz), '
          f'게이트 {len(course.gates)}개, 결승점 {np.round(finish, 2)}')
    dry_run(policy, observer, finish)
    if args.dry_run:
        return

    from crazyflie_interfaces.msg import LogDataGeneric
    from crazyflie_py import Crazyswarm
    from geometry_msgs.msg import Twist

    swarm = Crazyswarm()
    th = swarm.timeHelper
    node = swarm.allcfs
    cf = swarm.allcfs.crazyflies[0]
    buf = StateBuffer()
    node.create_subscription(LogDataGeneric, f'{cf.prefix}/rl_pv',
                             lambda m: buf.on_pv(m, th.time()), 10)
    node.create_subscription(LogDataGeneric, f'{cf.prefix}/rl_att',
                             lambda m: buf.on_att(m, th.time()), 10)
    node.create_subscription(LogDataGeneric, f'{cf.prefix}/rl_gyro',
                             lambda m: buf.on_gyro(m, th.time()), 10)
    pub = node.create_publisher(Twist, f'{cf.prefix}/cmd_vel_legacy', 10)

    def send(roll, pitch, yawrate, thrust):
        msg = Twist()
        msg.linear.y, msg.linear.x, msg.angular.z, msg.linear.z = roll, pitch, yawrate, thrust
        pub.publish(msg)

    # 상태 로그가 들어오는지 먼저 확인 (launch.py 의 rl_pv/rl_att/rl_gyro)
    t_wait = th.time()
    while not buf.ready() or buf.age(th.time()) > args.stale:
        th.sleep(0.05)
        if th.time() - t_wait > 5.0:
            print('[rl_flight] ✗ 상태 로그(rl_pv/rl_att/rl_gyro)가 오지 않는다. '
                  'crazyflie_rl launch 로 서버를 띄웠는지 확인')
            return
    if not args.shadow and not check_params(cf):
        return

    log_path = open_log(args)
    rec = start_record(cf, args) if args.record else None
    thrust_map = ThrustMap(args.hover_pwm)
    try:
        if args.shadow:
            run_policy(args, th, buf, policy, observer, finish, thrust_map, None, log_path)
            return
        # 1. 지상 thrust lock 해제 → HLC 이륙·정렬
        for _ in range(3):
            send(0.0, 0.0, 0.0, 0.0)
            th.sleep(0.02)
        cf.notifySetpointsStop()
        th.sleep(0.1)
        print(f'  이륙 → {finish[2]:.2f} m')
        cf.takeoff(targetHeight=float(finish[2]), duration=3.0)
        th.sleep(3.5)
        cf.goTo(finish, yaw=0.0, duration=3.0)
        th.sleep(3.5)

        # 2. 호버 PWM 보정
        if args.hover_pwm is None:
            pwm = []
            t0 = th.time()
            while th.time() - t0 < args.calib_time:
                pwm.append(np.mean(buf.state()[4]))
                th.sleepForRate(policy.control_hz)
            hover = float(np.mean(pwm))
            if not 15000 < hover < 60000:
                print(f'  ✗ 호버 PWM {hover:.0f} 이 비정상 — 착륙')
                land(cf, th, buf)
                return
            thrust_map = ThrustMap(hover)
            print(f'  호버 PWM {hover:.0f} → 학습 호버 추력 {SIM_MASS * G:.3f} N 에 맞춤')

        p, v, _, _, _ = buf.state()
        if np.linalg.norm(p - finish) > 0.15 or np.linalg.norm(v) > 0.2:
            print(f'  ✗ start 상공에 안정되지 않았다 (오차 {np.linalg.norm(p - finish):.2f} m, '
                  f'속도 {np.linalg.norm(v):.2f} m/s) — 착륙')
            land(cf, th, buf)
            return

        # 3. 정책 비행
        run_policy(args, th, buf, policy, observer, finish, thrust_map, send, log_path)
        # 4. HLC 로 넘겨 착륙
        land(cf, th, buf)
    finally:
        if rec is not None:
            rec.stop()
        print(f'  기록 → {log_path}')


def run_policy(args, th, buf, policy, observer, finish, thrust_map, send, log_path):
    """100 Hz 정책 루프. send=None 이면 섀도 모드(명령 안 보냄)."""
    hz = policy.control_hz
    lo, hi = gc.load_course(args.gates).room.bounds()
    cos_tilt = math.cos(math.radians(args.max_tilt))
    prev_action = np.zeros(4)
    settle = 0
    t0 = th.time()
    mode = '섀도' if send is None else '정책 비행'
    print(f'  ▶ {mode} 시작 (최대 {args.max_time:.0f} s)')
    with open(log_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'qx', 'qy', 'qz', 'qw',
                    'wx', 'wy', 'wz', 'm1', 'm2', 'm3', 'm4', 'gate',
                    'a0', 'a1', 'a2', 'a3', 'cmd_wx', 'cmd_wy', 'cmd_wz', 'cmd_T', 'base_pwm'])
        while not th.isShutdown():
            now = th.time()
            t = now - t0
            pos, vel, quat, omega, pwm = buf.state()
            reason = None
            if buf.age(now) > args.stale:
                reason = f'상태 로그 끊김 {buf.age(now) * 1000:.0f} ms'
            elif quat_to_matrix(quat)[2, 2] < cos_tilt:
                reason = f'기울기 > {args.max_tilt:.0f}°'
            elif np.any(pos[:2] < lo[:2] + args.wall_margin) or np.any(pos[:2] > hi[:2] - args.wall_margin):
                reason = '방 경계 접근'
            elif pos[2] < args.min_z or pos[2] > hi[2] - 0.1:
                reason = f'고도 {pos[2]:.2f} m'
            elif t > args.max_time:
                reason = '시간 초과'
            if reason and send is not None:
                print(f'  ✗ 중단: {reason} (t={t:.2f} s, 게이트 {observer.gate}/7)')
                return False

            gid = observer.update(pos)
            if gid is not None:
                print(f'  G{gid} 통과 t={t:.2f} s, 속도 {np.linalg.norm(vel):.2f} m/s', flush=True)
            obs = observer.observe(pos, vel, quat, omega, thrust_map.motor_force(pwm), prev_action)
            action = policy.act(obs)
            cmd = policy.command(action)
            legacy = to_legacy(cmd, thrust_map)
            if send is not None:
                send(*legacy)
            prev_action = action
            w.writerow([f'{t:.3f}', *np.round(pos, 4), *np.round(vel, 4), *np.round(quat, 5),
                        *np.round(omega, 4), *pwm.astype(int), observer.gate,
                        *np.round(action, 4), *np.round(cmd, 4), int(legacy[3])])

            near = np.linalg.norm(pos - finish) < 0.1 and np.linalg.norm(vel) < 0.2
            settle = settle + 1 if (observer.gate == 7 and near) else 0
            if settle >= round(0.5 * hz) + 1:
                print(f'  ✓ 완주: 결승점 정착 t={t:.2f} s')
                return True
            th.sleepForRate(hz)
    return False


def land(cf, th, buf):
    """스트리밍 중단 → HLC 로 현재 위치 유지 → 착륙."""
    cf.notifySetpointsStop()
    th.sleep(0.05)
    pos = buf.state()[0].copy()
    pos[2] = max(pos[2], 0.3)
    cf.goTo(pos, yaw=0.0, duration=1.5)
    th.sleep(2.0)
    print('  착륙')
    cf.land(targetHeight=0.04, duration=3.0)
    th.sleep(3.5)


def check_params(cf):
    """PID 제어기·RPYT rate 모드인지 확인. 아니면 비행하지 않는다."""
    want = {'stabilizer.controller': 1, 'flightmode.stabModeRoll': 0,
            'flightmode.stabModePitch': 0, 'flightmode.stabModeYaw': 0}
    ok = True
    for name, value in want.items():
        got = cf.getParam(name)
        if got != value:
            print(f'  ✗ 파라미터 {name} = {got} (필요 {value}) — crazyflie_rl launch 설정 확인')
            ok = False
    return ok


def open_log(args):
    root = os.path.expanduser(args.log_dir)
    os.makedirs(root, exist_ok=True)
    tag = 'shadow' if args.shadow else 'flight'
    return os.path.join(root, f'rl_{tag}_{time.strftime("%Y%m%d_%H%M%S")}.csv')


def start_record(cf, args):
    from crazyflie_test.recorder import Recorder
    bag_dir = os.path.join(os.path.expanduser(args.log_dir),
                           'rl_' + time.strftime('%Y%m%d_%H%M%S'))
    return Recorder(bag_dir, ['/poses', f'{cf.prefix}/cmd_vel_legacy'] + [
        f'{cf.prefix}/{t}' for t in ('rl_pv', 'rl_att', 'rl_gyro', 'status')])


if __name__ == '__main__':
    main()
