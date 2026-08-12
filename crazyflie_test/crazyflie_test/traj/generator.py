"""오프라인 궤적 CSV 생성 + 검증 플롯 (비행과 별개, 다른 시뮬/분석용).

브리프 스키마로 **한 랩(period)** 의 주기 레퍼런스를 낸다(램프 없음 — 순수 루프).
비행(`flight.py`)은 여기에 시작/종료 램프를 더한 것이라 별도다.

    python3 -m crazyflie_test.traj.generator --shape clover --speed 2.0 --yaw forward
    python3 -m crazyflie_test.traj.generator --all --speed 1.0 --plot

CSV 스키마: t,x,y,z,vx,vy,vz,ax,ay,az,yaw,yawrate
"""
import argparse
import os

import numpy as np

from . import shapes as sh

CSV_HEADER = 't,x,y,z,vx,vy,vz,ax,ay,az,yaw,yawrate'


def generate(shape_fn, max_speed, rate_hz=100, yaw_mode='forward'):
    """한 랩의 주기 레퍼런스. 속도 격리: period 만 조절해 max_speed 를 맞춘다."""
    unit = sh.max_speed_unit(shape_fn)
    T = unit / max(max_speed, 1e-6)               # T = max|dp/ds| / v
    N = max(2, int(round(T * rate_hz)))
    t = np.arange(N) / rate_hz
    s = t / T                                     # [0, 1)
    pos, d1, d2 = shape_fn(s)
    vel = d1 / T
    acc = d2 / (T * T)
    if yaw_mode == 'forward':
        yaw = np.unwrap(np.arctan2(vel[:, 1], vel[:, 0]))
        yawrate = np.gradient(yaw, t)
    else:
        yaw = np.zeros(N)
        yawrate = np.zeros(N)
    return {'t': t, 'pos': pos, 'vel': vel, 'acc': acc,
            'yaw': yaw, 'yawrate': yawrate, 'period': T}


def save_csv(traj, path):
    cols = np.column_stack([
        traj['t'], traj['pos'], traj['vel'], traj['acc'],
        traj['yaw'], traj['yawrate']])
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    np.savetxt(path, cols, delimiter=',', header=CSV_HEADER, comments='', fmt='%.6f')


def _vtag(v):
    return f'{v:.1f}'.replace('.', 'p')


def main():
    p = argparse.ArgumentParser(description='주기 궤적 CSV 생성 (오프라인)')
    p.add_argument('--shape', default=None, help='도형 하나')
    p.add_argument('--all', action='store_true', help='5개 도형 모두')
    p.add_argument('--speed', type=float, default=1.0, help='목표 최대 속도 [m/s]')
    p.add_argument('--yaw', choices=['forward', 'constant'], default='forward')
    p.add_argument('--rate', type=float, default=100.0, help='샘플링 [Hz]')
    p.add_argument('--out', default='traj_ref', help='출력 폴더')
    p.add_argument('--plot', action='store_true', help='top-down 검증 플롯 저장')
    args = p.parse_args()

    names = sh.shape_names() if args.all else [args.shape or 'clover']
    yaw_type = 'yawForward' if args.yaw == 'forward' else 'yawConstant'
    trajs = {}
    for name in names:
        fn = sh.get_shape(name)
        tj = generate(fn, args.speed, args.rate, args.yaw)
        out = os.path.join(args.out, name, yaw_type,
                           f'{name}_maxSpeed{_vtag(args.speed)}.csv')
        save_csv(tj, out)
        realized = float(np.max(np.linalg.norm(tj['vel'], axis=1)))
        loop = float(np.linalg.norm(tj['pos'][0] - tj['pos'][-1]))
        print(f'{name:8s} period={tj["period"]:.2f}s N={len(tj["t"])} '
              f'실현속도={realized:.3f}(목표{args.speed}) 루프오차={loop:.3e} → {out}')
        trajs[name] = tj

    if args.plot:
        _plot(trajs, os.path.join(args.out, 'shapes_top.png'))


def _plot(trajs, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    n = len(trajs)
    fig, axs = plt.subplots(1, n, figsize=(4 * n, 4), squeeze=False)
    for ax, (name, tj) in zip(axs[0], trajs.items()):
        p = tj['pos']
        ax.plot(p[:, 0], p[:, 1], 'b-')
        ax.set_aspect('equal')
        ax.grid(alpha=0.3)
        ax.set_title(f'{name} (z {p[:,2].min():.2f}~{p[:,2].max():.2f})')
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.savefig(path, dpi=90, bbox_inches='tight')
    print(f'[plot] {path}')


if __name__ == '__main__':
    main()
