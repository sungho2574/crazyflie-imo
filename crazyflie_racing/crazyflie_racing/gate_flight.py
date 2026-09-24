"""게이트 코스 비행 — 미리 계획한 궤적을 **크플 펌웨어 온보드 추종기**로 실행한다.

    ros2 run crazyflie_racing gate_flight --dry-run      # 궤적 요약·검사만 (기체 불필요)
    ros2 run crazyflie_racing gate_flight                # 실행 (timescale 1.0)
    ros2 run crazyflie_racing gate_flight --timescale 2  # 절반 속도 (첫 비행 권장)
    ros2 run crazyflie_racing gate_flight --laps 3       # 같은 궤적을 3바퀴 반복 (바퀴마다 정지)
    ros2 run crazyflie_racing gate_flight --loop 10      # 진입→순환×10→탈출, 멈추지 않고 연속
    ros2 run crazyflie_racing gate_flight --record       # 이륙~착륙 rosbag 기록 (~/flight_logs/gate_*)

경로 계획은 이 스크립트가 하지 않는다. TOGT-Planner 로 **오프라인**에서 시간최적
궤적을 만들어 다항식 CSV(`config/gate_trajectory.csv`)로 저장해 두고
(`plan_gate_trajectory` 참고), 여기서는 그 궤적을 펌웨어 High-Level Commander 에
업로드해 실행만 한다. 이 방식은 crazyswarm2 의 `figure8.py`, `multi_trajectory.py`
와 같은 표준이며, 펌웨어가 온보드에서 500 Hz+ 로 추종하므로 통신이 끊겨도 궤적을
계속 따라간다(직접 cmdFullState 를 스트리밍하지 않는다).

    이륙(start 상공) → goTo(궤적 시작점) → uploadTrajectory →
    startTrajectory → 궤적 시간 대기 → land

⚠️ 궤적은 gates.yaml 의 start 상공에서 시작하도록 계획돼 있다. 이륙 후 그 지점에
   goTo 로 정렬한 뒤 startTrajectory 를 부르므로, 궤적이 절대 world 좌표(게이트
   위치)를 그대로 따른다. 이를 위해 기체의 initial_position 이 gates.yaml 의
   start 와 같아야 한다(gate_flight 가 실행 전에 검사한다).

실기체 체크리스트
    1. 기체를 gates.yaml 의 start 좌표에, 기수 +x 로 놓는다. 이·착륙 지점은 같다.
    2. crazyflies_*.yaml 의 initial_position 을 그 좌표로 맞춘다(다르면 이륙 거부).
    3. 배터리·tumble·lock 자동 점검. 첫 비행은 --timescale 크게(느리게).
    4. 조종기(teleop)의 emergency 를 항상 손 닿는 곳에.
"""
import argparse
import os
import sys
import time

import numpy as np

from . import gate_course as gc

MIN_BATTERY = 3.85      # V  이 아래면 코스를 완주하기 어렵다
START_TOL = 0.05        # m  initial_position 과 gates.yaml start 의 허용 오차


def parse_args():
    p = argparse.ArgumentParser(description='게이트 코스 비행 (펌웨어 온보드 궤적 실행)')
    p.add_argument('--trajectory', default=None,
                   help='궤적 CSV (기본: 패키지 config/gate_trajectory.csv)')
    p.add_argument('--gates', default=None,
                   help='gates.yaml (start·검사용, 기본: 패키지 config)')
    p.add_argument('--timescale', type=float, default=1.0,
                   help='궤적 시간 배율. >1 이면 느리게 (첫 비행 권장). 예: 2 = 절반 속도')
    p.add_argument('--laps', type=int, default=1,
                   help='같은 궤적을 반복할 바퀴 수. 바퀴 사이에 start 상공에서 잠깐 멈춘다')
    p.add_argument('--lap-pause', type=float, default=1.0,
                   help='바퀴 사이 start 상공 재정렬 시간 [s]')
    p.add_argument('--loop', type=int, default=0,
                   help='연속 바퀴 수. config/gate_loop_{entry,lap,exit}.csv 를 이어 붙여 '
                        '바퀴 사이에 멈추지 않고 돈다 (0 = 안 씀)')
    p.add_argument('--height', type=float, default=None,
                   help='이륙 고도 [m]. 기본은 gates.yaml 의 start.takeoff_z')
    p.add_argument('--takeoff-duration', type=float, default=3.0, help='이륙 시간 [s]')
    p.add_argument('--land-duration', type=float, default=3.0, help='착륙 시간 [s]')
    p.add_argument('--min-battery', type=float, default=MIN_BATTERY,
                   help='이 전압 미만이면 이륙하지 않는다 [V]. 0 이면 검사 안 함')
    p.add_argument('--force-start', action='store_true',
                   help='initial_position 이 gates.yaml start 와 달라도 강행')
    p.add_argument('--mocap', action='store_true',
                   help='모캡(Qualisys)으로 절대 위치를 받는 경우. initial_position=start '
                        '검사를 건너뛴다(원점은 mocap 캘리브레이션이 정한다). '
                        '⚠️ mocap world 원점이 gates.yaml 원점(방 중심)과 같아야 한다')
    p.add_argument('--no-arm', action='store_true', help='arm 요청을 보내지 않는다')
    p.add_argument('--record', action='store_true',
                   help='이륙 직전~착륙까지 rosbag 기록 (/poses, pose, imu_raw, motor_pwm, status)')
    p.add_argument('--record-dir', default='~/flight_logs',
                   help='기록 폴더. 그 아래 gate_<날짜_시각>/ 로 저장 (analysis/gate_3d.py 가 읽는 형식)')
    p.add_argument('--dry-run', action='store_true', help='계획만 출력하고 비행하지 않음')
    args, _ = p.parse_known_args()
    return args


# --------------------------------------------------------------------------- #
# 궤적 로딩·검사
# --------------------------------------------------------------------------- #
def load_and_report(args):
    """궤적 CSV 를 읽고 검사 결과를 출력. (traj, course, hover) 반환.

    traj 는 crazyflie_py 의 Trajectory (없으면 None — dry-run 은 CSV 샘플만으로 진행).
    """
    course = gc.load_course(args.gates)
    hover = course.start_hover(args.height)
    if args.loop:
        if args.laps > 1 or args.trajectory:
            sys.exit('[gate_flight] --loop 은 --laps / --trajectory 와 같이 쓸 수 없다')
        paths = gc.default_loop_paths()
        parts = {k: gc.load_rows(v) for k, v in paths.items()}
        # 기하 검사는 진입+1랩+탈출로 충분하다(랩은 같은 경로를 반복)
        rows = np.vstack([parts[k] for k in gc.LOOP_PARTS])
        csv = ', '.join(os.path.basename(paths[k]) for k in gc.LOOP_PARTS)
    else:
        csv = args.trajectory or gc.default_trajectory_path()
        rows = gc.load_rows(csv)

    pts = gc.sample_rows(rows)
    n_pieces = len(rows)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    info = gc.preview(pts, course, home_xy=course.start[:2])
    start_pt = pts[0]

    print(f'[gate_flight] 궤적 {csv}')
    print(f'  게이트 {len(course.gates)}개, 통과 순서 '
          + ' → '.join(f'G{g.id}' for g in course.gates))
    print(f'  이륙/착륙 지점 ({course.start[0]:+.2f}, {course.start[1]:+.2f}), '
          f'호버 고도 {hover[2]:.2f} m')
    print(f'  궤적 시작점 ({start_pt[0]:+.2f}, {start_pt[1]:+.2f}, {start_pt[2]:.2f})')
    print(f'  [check] 조각 수      : {n_pieces} (펌웨어 메모리 한계 ~32 이하) -> '
          f'{"OK" if n_pieces <= 32 else "확인 필요"}')
    if args.loop:
        # 이음매에서 위치·속도·가속도가 같아야 끊김 없이 이어진다
        seams = [('entry', 'lap'), ('lap', 'lap'), ('lap', 'exit')]
        for a, b in seams:
            dp, dv, da = gc.seam_error(parts[a], parts[b])
            ok = dp < 1e-3 and dv < 1e-2 and da < 1e-1
            print(f'  [check] 이음매 {a:>5}→{b:<5}: Δpos {dp:.1e} Δvel {dv:.1e} '
                  f'Δacc {da:.1e} -> {"OK" if ok else "NG — 이어지지 않는다"}')
    print(f'  [check] path x/y/z   : x[{lo[0]:.2f},{hi[0]:.2f}] '
          f'y[{lo[1]:.2f},{hi[1]:.2f}] z[{lo[2]:.2f},{hi[2]:.2f}]')
    print(f'  [check] 벽까지 여유  : {info["wall"]:.2f} m '
          f'(기준 {course.room.safety_margin:.2f}) -> '
          f'{"OK" if info["wall"] >= course.room.safety_margin else "NG"}')
    print(f'  [check] 방 이탈      : {info["escaped"]} 점 -> '
          f'{"OK" if not info["escaped"] else "NG"}')
    edge = gc.gate_edge_clearance(pts, course)
    worst = min(edge.values()) if edge else float('inf')
    print(f'  [check] 프레임 여유  : 최소 {worst:.3f} m (목표 {course.frame_margin:.2f} m, '
          f'프로펠러 끝 기준) -> {"OK" if worst >= course.frame_margin - 0.01 else "NG"}')
    hits = gc.check_gate_clearance(pts, course)
    for h in hits:
        print(f'            {h}')
    for w in info['warnings']:
        print(f'  ⚠ {w}')

    if info['escaped'] or [h for h in hits if '충돌' in h]:
        print('  ⚠ 궤적이 방을 벗어나거나 프레임과 충돌한다. '
              'plan_gate_trajectory 로 다시 계획할 것')

    if args.loop:
        dur = {k: float(parts[k][:, 0].sum()) * args.timescale for k in gc.LOOP_PARTS}
        total = dur['entry'] + args.loop * dur['lap'] + dur['exit']
        print(f'  연속 {args.loop}바퀴: 진입 {dur["entry"]:.1f} s + 랩 {dur["lap"]:.1f} s × '
              f'{args.loop} + 탈출 {dur["exit"]:.1f} s = {total:.1f} s '
              f'(timescale {args.timescale:.2f})')
        try:
            traj = {k: gc.load_trajectory(v) for k, v in paths.items()}
        except Exception as exc:  # crazyflie_py 없는 환경(순수 dry-run)
            print(f'  · Trajectory 로드는 건너뜀({exc}). CSV 검사만 수행.')
            traj = None
        return traj, course, hover

    traj = None
    try:
        traj = gc.load_trajectory(csv)
        lap_t = traj.duration * args.timescale
        print(f'  궤적 시간 {traj.duration:.1f} s '
              f'(timescale {args.timescale:.2f} → {lap_t:.1f} s)')
        if args.laps > 1:
            total = args.laps * lap_t + (args.laps - 1) * (args.lap_pause + 0.5)
            print(f'  {args.laps}바퀴 반복 → 약 {total:.1f} s '
                  f'(바퀴 사이 start 상공 재정렬 {args.lap_pause:.1f} s)')
    except Exception as exc:      # crazyflie_py 없는 환경(순수 dry-run)
        print(f'  · Trajectory 로드는 건너뜀({exc}). CSV 검사만 수행.')
    return traj, course, hover


# --------------------------------------------------------------------------- #
# 실기체 사전 점검
# --------------------------------------------------------------------------- #
def preflight(swarm, cf, course, args):
    """이륙 전에 확인할 것들. 문제가 있으면 False."""
    from crazyflie_interfaces.msg import Status

    want = course.start
    if args.mocap:
        # 모캡은 절대 위치를 트래커가 준다 → initial_position 은 원점이 아니다.
        # 대신 mocap world 원점이 gates.yaml 원점과 같아야 한다(캘리브레이션 책임).
        print('  모캡 모드: 위치는 Qualisys 가 절대좌표로 준다(initial_position 검사 생략).'
              '\n    ⚠️ mocap world 원점(캘리브레이션 L프레임)이 gates.yaml 원점(방 중심)과'
              '\n       같고 x/y 축이 정렬돼 있어야 게이트 좌표가 맞는다.')
    else:
        init = np.array(cf.initialPosition, dtype=float)
        err = float(np.linalg.norm(init[:2] - want[:2]))
        print(f'  initial_position {np.round(init, 2)} vs gates.yaml start '
              f'({want[0]:+.2f}, {want[1]:+.2f}) → 차이 {err:.3f} m')
        if err > START_TOL:
            print('  ✗ 두 값이 다르다. 기체는 initial_position 을 원점 삼아 위치를 추정하므로,'
                  '\n    이대로 날리면 게이트 좌표가 통째로 어긋난다. 다음 중 하나로 맞출 것:'
                  f'\n      · crazyflies_*.yaml 의 initial_position 을 '
                  f'[{want[0]}, {want[1]}, 0.0] 으로 수정 (권장)'
                  '\n      · gates.yaml 의 start 를 기체 위치로 수정'
                  '\n    모캡이면 --mocap, 검사만 건너뛰려면 --force-start')
            if not args.force_start:
                return False

    if args.min_battery <= 0.0:
        return True

    # /<name>/status 를 잠깐 받아 배터리·supervisor 상태를 본다 (sim 에는 없을 수 있음)
    state = {}

    def on_status(msg):
        state['msg'] = msg

    node = swarm.allcfs
    sub = node.create_subscription(Status, f'{cf.prefix}/status', on_status, 1)
    for _ in range(30):
        swarm.timeHelper.sleep(0.1)
        if 'msg' in state:
            break
    node.destroy_subscription(sub)

    if 'msg' not in state:
        print('  · status 토픽이 없어 배터리·arm 상태를 확인하지 못했다 '
              '(sim 이면 정상. 실기체면 firmware_logging 의 status 를 켤 것)')
        return True

    msg = state['msg']
    info = msg.supervisor_info
    print(f'  배터리 {msg.battery_voltage:.2f} V (참고용, 통과 조건 아님), '
          f'supervisor 0b{info:07b}')
    if info & Status.SUPERVISOR_INFO_IS_TUMBLED:
        print('  ✗ 기체가 뒤집혀 있다(tumbled). 바로 놓고 다시 할 것')
        return False
    if info & Status.SUPERVISOR_INFO_IS_LOCKED:
        print('  ✗ supervisor lock 상태다(보통 추락 후). 재부팅이 필요하다:'
              '\n      ros2 run crazyflie reboot --uri <URI>   (서버를 먼저 끌 것)')
        return False
    if not args.no_arm and not (info & Status.SUPERVISOR_INFO_IS_ARMED):
        if info & Status.SUPERVISOR_INFO_CAN_BE_ARMED:
            print('  · arm 요청')
            cf.arm(True)
            swarm.timeHelper.sleep(1.0)
        else:
            print('  · 아직 arm 할 수 없는 상태다. 그대로 진행한다 '
                  '(구형 펌웨어면 arm 자체가 없으니 정상)')
    return True


# --------------------------------------------------------------------------- #
# 비행
# --------------------------------------------------------------------------- #
def main():
    args = parse_args()
    traj, course, hover = load_and_report(args)
    if args.dry_run:
        return
    if traj is None:
        print('[gate_flight] 궤적을 로드하지 못해 비행할 수 없다')
        return

    from crazyflie_py import Crazyswarm      # dry-run 은 기체 없이도 되게 늦게 import

    swarm = Crazyswarm()
    th = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    if not preflight(swarm, cf, course, args):
        print('[gate_flight] 사전 점검 실패 — 이륙하지 않는다')
        return

    first = traj['entry'] if args.loop else traj
    start_pt = first.eval(0.0).pos  # 궤적의 절대 시작점 (호버 고도)

    rec = None
    if args.record:
        from crazyflie_test.recorder import Recorder
        bag_dir = os.path.join(os.path.expanduser(args.record_dir),
                               'gate_' + time.strftime('%Y%m%d_%H%M%S'))
        # /poses = mocap GT (모캡 모드에서만 존재)
        rec = Recorder(bag_dir, ['/poses'] + [
            f'{cf.prefix}/{t}' for t in ('pose', 'imu_raw', 'motor_pwm', 'status')])
        th.sleep(2.0)                # 레코더가 구독을 붙일 시간
    try:
        if args.loop:
            fly_loop(cf, th, traj, hover, start_pt, args)
        else:
            fly(cf, th, traj, hover, start_pt, args)
    finally:                         # Ctrl+C 로 끊겨도 bag 은 닫는다
        if rec is not None:
            rec.stop()


def fly(cf, th, traj, hover, start_pt, args):
    """이륙 → 시작점 정렬 → 궤적 N바퀴 → 착륙."""
    print(f'  이륙 → {hover[2]:.2f} m')
    cf.takeoff(targetHeight=hover[2], duration=args.takeoff_duration)
    th.sleep(args.takeoff_duration + 1.0)

    # 궤적 시작점으로 정렬. 궤적의 시작점(=start 상공)에 정확히 가 있으면,
    # relative=True 로 실행해도 절대좌표와 동일하다(shift ≈ 0). figure8.py 등
    # crazyswarm2 예제가 쓰는 검증된 방식이라 이쪽을 쓴다.
    cf.goTo(start_pt, yaw=0.0, duration=3.0)
    th.sleep(3.5)

    cf.uploadTrajectory(0, 0, traj)
    laps = max(1, args.laps)
    for lap in range(1, laps + 1):
        if lap > 1:
            # 궤적은 start 상공에서 속도 0 으로 끝난다. relative=True 는 현재 위치 기준이라
            # 끝 오차가 다음 바퀴로 누적되므로, 매 바퀴 start 상공에 다시 정렬한다.
            cf.goTo(start_pt, yaw=0.0, duration=args.lap_pause)
            th.sleep(args.lap_pause + 0.5)
        print(f'  랩 {lap}/{laps}', flush=True)
        cf.startTrajectory(0, timescale=args.timescale, relative=True)
        th.sleep(traj.duration * args.timescale + (1.0 if lap == laps else 0.5))

    print('  착륙')
    cf.land(targetHeight=0.04, duration=args.land_duration)
    th.sleep(args.land_duration + 1.0)



def fly_loop(cf, th, trajs, hover, start_pt, args):
    """이륙 → 시작점 정렬 → 진입 → 순환 랩 × N → 탈출 → 착륙 (멈춤 없음).

    세 궤적을 펌웨어 메모리에 연달아 올려 두고, 각 궤적이 끝나는 시각에 다음 궤적을
    시작한다. 펌웨어는 궤적을 줄 세우지 못하므로 전환 시각은 호스트가 맞춘다 — 시각은
    처음 시작 기준 절대시각으로 잡아 sleep 오차가 누적되지 않게 한다. 이음매에서
    pos/vel/acc 가 같으므로 전환 명령이 제때 들어가면 끊김이 없고, 무선 지연(수~수십 ms)
    만큼만 살짝 어긋난다.
    ⚠️ 모두 relative=False(절대좌표). relative=True 는 '현재 setpoint' 기준으로 옮기므로
       전환이 조금만 일러도 그 차이가 이후 모든 랩에 남는다.
    """
    print(f'  이륙 → {hover[2]:.2f} m')
    cf.takeoff(targetHeight=hover[2], duration=args.takeoff_duration)
    th.sleep(args.takeoff_duration + 1.0)
    cf.goTo(start_pt, yaw=0.0, duration=3.0)
    th.sleep(3.5)

    ids = {}
    offset = 0
    for tid, part in enumerate(gc.LOOP_PARTS):   # 조각 메모리에 이어서 배치
        cf.uploadTrajectory(tid, offset, trajs[part])
        ids[part] = tid
        offset += len(trajs[part].polynomials)

    ts = args.timescale
    t_entry = trajs['entry'].duration * ts
    t_lap = trajs['lap'].duration * ts
    t_exit = trajs['exit'].duration * ts

    def wait_until(t):
        th.sleep(max(0.0, t - th.time()))

    t0 = th.time()
    cf.startTrajectory(ids['entry'], timescale=ts, relative=False)
    for lap in range(args.loop):
        wait_until(t0 + t_entry + lap * t_lap)
        cf.startTrajectory(ids['lap'], timescale=ts, relative=False)
        print(f'  랩 {lap + 1}/{args.loop}', flush=True)
    wait_until(t0 + t_entry + args.loop * t_lap)
    cf.startTrajectory(ids['exit'], timescale=ts, relative=False)
    wait_until(t0 + t_entry + args.loop * t_lap + t_exit + 1.0)

    print('  착륙')
    cf.land(targetHeight=0.04, duration=args.land_duration)
    th.sleep(args.land_duration + 1.0)


if __name__ == '__main__':
    main()
