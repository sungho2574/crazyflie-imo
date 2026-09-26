"""정책 오프라인 점검 — 간이 강체 시뮬레이션으로 게이트를 도는지 확인 (ROS·기체 불필요).

    ros2 run crazyflie_rl rl_sim_check                    # 기본
    ros2 run crazyflie_rl rl_sim_check --latency 0.02     # 관측 20 ms 지연 (무선 지연 흉내)
    python3 -m crazyflie_rl.sim_check --plot out.png

목적은 **관측·행동 변환(racing_obs / policy)이 학습 환경과 맞는지** 보는 것이다. 이게
틀리면(축 부호·단위·게이트 순서 등) 정책은 첫 게이트도 못 지난다. 동역학은 crazyflow
cf2x_L250 파라미터(질량·관성·팔 길이·추력 곡선·항력)를 쓰되, 각속도 제어기와 모터는
1차 지연으로 단순화했다 — 여기서 통과하는 것은 필요조건이지 실기체 성공 보장이 아니다.
"""
import argparse

import numpy as np

from crazyflie_racing import gate_course as gc

from .policy import MOTOR_THRUST_MAX, MOTOR_THRUST_MIN, RacingPolicy
from .racing_obs import RacingObserver

G = 9.81
MASS = 0.0319
J = np.diag([16.8e-6, 16.8e-6, 29.8e-6])
ARM = 0.03253
YAW_K = 1.4592584373980652e-12 / 2.4582929831265485e-10      # 반토크/추력 비 [m]
DRAG = np.diag([-0.01471782, -0.01471782, -0.01277641])      # 기체 좌표계 항력 [N/(m/s)]
# 모터 m1(앞오른) m2(뒤오른) m3(뒤왼) m4(앞왼) — crazyflow mixing_matrix 와 같은 순서
MIX = np.array([[-1., -1., 1., 1.], [-1., 1., 1., -1.], [-1., 1., -1., 1.]])
ALLOC = np.vstack([np.ones(4), ARM * MIX[0], ARM * MIX[1], YAW_K * MIX[2]])


def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def expm_so3(w):
    th = np.linalg.norm(w)
    if th < 1e-9:
        return np.eye(3) + skew(w)
    k = skew(w / th)
    return np.eye(3) + np.sin(th) * k + (1 - np.cos(th)) * k @ k


def matrix_to_quat(r):
    """회전행렬 → xyzw."""
    w = np.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = np.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2
    y = np.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2
    z = np.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2
    x = np.copysign(x, r[2, 1] - r[1, 2])
    y = np.copysign(y, r[0, 2] - r[2, 0])
    z = np.copysign(z, r[1, 0] - r[0, 1])
    return np.array([x, y, z, w])


def rollout(policy, course, finish, latency=0.0, rate_tau=0.03, motor_tau=0.03,
            physics_hz=500, max_time=20.0):
    obs_mgr = RacingObserver(course, finish)
    hz = policy.control_hz
    sub = physics_hz // hz
    dt = 1.0 / physics_hz
    delay = int(round(latency * hz))

    p = np.array(finish, dtype=float)
    v = np.zeros(3)
    r = np.eye(3)
    w = np.zeros(3)
    f = np.full(4, MASS * G / 4)
    prev_action = np.zeros(4)
    history = []                                   # 관측 지연용 과거 상태
    log = {'t': [], 'pos': [], 'gate': [], 'cmd': []}
    events = []
    settle = 0
    lo_room, hi_room = course.room.bounds()

    for k in range(int(max_time * hz)):
        history.append((p.copy(), v.copy(), r.copy(), w.copy(), f.copy()))
        sp, sv, sr, sw, sf = history[max(0, len(history) - 1 - delay)]
        obs = obs_mgr.observe(sp, sv, matrix_to_quat(sr), sw, sf, prev_action)
        action = policy.act(obs)
        cmd = policy.command(action)
        prev_action = action

        for _ in range(sub):
            # 각속도 1차 추종 → 필요 토크, 추력·토크 → 모터 분배 → 모터 1차 지연
            tau = J @ ((cmd[:3] - w) / rate_tau) + np.cross(w, J @ w)
            f_des = np.clip(np.linalg.solve(ALLOC, np.r_[cmd[3], tau]),
                            MOTOR_THRUST_MIN, MOTOR_THRUST_MAX)
            f += (f_des - f) * min(1.0, dt / motor_tau)
            wrench = ALLOC @ f
            force = r @ (np.array([0, 0, wrench[0]]) + DRAG @ (r.T @ v)) + np.array([0, 0, -MASS * G])
            v = v + force / MASS * dt
            p = p + v * dt
            w = w + np.linalg.solve(J, wrench[1:] - np.cross(w, J @ w)) * dt
            r = r @ expm_so3(w * dt)
            gid = obs_mgr.update(p)
            if gid is not None:
                events.append((gid, (k + 1) / hz))

        t = (k + 1) / hz
        log['t'].append(t)
        log['pos'].append(p.copy())
        log['gate'].append(obs_mgr.gate)
        log['cmd'].append(cmd.copy())
        if r[2, 2] < 0.3 or np.any(p < lo_room) or np.any(p > hi_room) or not np.all(np.isfinite(p)):
            return 'crash', t, events, log
        near = np.linalg.norm(p - finish) < 0.1 and np.linalg.norm(v) < 0.2
        settle = settle + 1 if (obs_mgr.gate == 7 and near) else 0
        if settle >= round(0.5 * hz) + 1:
            return 'success', t, events, log
    return 'timeout', max_time, events, log


def main():
    ap = argparse.ArgumentParser(description='레이싱 정책 오프라인 점검 (간이 시뮬레이션)')
    ap.add_argument('--model-dir', default=None)
    ap.add_argument('--gates', default=None, help='gates.yaml (기본: crazyflie_racing config)')
    ap.add_argument('--latency', type=float, default=0.0, help='관측 지연 [s]')
    ap.add_argument('--rate-tau', type=float, default=0.03, help='각속도 추종 시정수 [s]')
    ap.add_argument('--motor-tau', type=float, default=0.03, help='모터 시정수 [s]')
    ap.add_argument('--plot', default=None, help='궤적 그림 저장 경로 (matplotlib 필요)')
    args, _ = ap.parse_known_args()

    policy = RacingPolicy(args.model_dir)
    course = gc.load_course(args.gates)
    finish = course.start_hover()
    result, t, events, log = rollout(policy, course, finish, args.latency,
                                     args.rate_tau, args.motor_tau)
    pos = np.array(log['pos'])
    cmd = np.array(log['cmd'])
    speed = np.linalg.norm(np.diff(pos, axis=0), axis=1) * policy.control_hz
    print(f'[rl_sim_check] 결과: {result} ({t:.2f} s), 지연 {args.latency * 1000:.0f} ms')
    print('  게이트 통과: ' + (', '.join(f'G{g}@{s:.2f}s' for g, s in events) or '없음'))
    print(f'  최대 속도 {speed.max():.2f} m/s, 명령 각속도 최대 '
          f'{np.abs(cmd[:, :3]).max(axis=0).round(1)} rad/s, 총추력 '
          f'[{cmd[:, 3].min():.3f}, {cmd[:, 3].max():.3f}] N (호버 {MASS * G:.3f})')
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(6, 8))
        ax.plot(pos[:, 0], pos[:, 1], lw=1)
        for g in course.gates:
            c, lat = g.center, g.lateral * g.inner_size / 2
            ax.plot([c[0] - lat[0], c[0] + lat[0]], [c[1] - lat[1], c[1] + lat[1]], 'g-', lw=3)
            ax.annotate(f'G{g.id}', c[:2])
        ax.set_aspect('equal')
        ax.set_title(f'{result} {t:.1f}s')
        fig.savefig(args.plot, dpi=120)
        print(f'  그림 → {args.plot}')


if __name__ == '__main__':
    main()
