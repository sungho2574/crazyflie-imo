"""8자 궤적 비행 — 다항식 궤적 업로드 방식 (정석·부드러움).

crazyswarm2 표준 figure8.csv(10구간 조각다항식, ~7.3s)를 기체에 업로드해
high-level commander 가 연속 궤적을 추종한다. goTo 샘플링보다 훨씬 부드럽고,
mocap 으로 넘어가도 그대로 쓸 수 있다.

기본 형상은 원점 중심 x ±1.0m, y ±0.54m (z=0, 시작 위치 기준 상대 재생).
크기와 속도는 실행 옵션으로 조절한다:

    ros2 run crazyflie_test figure8_traj --scale 2.0 --timescale 2.0

⚠️ --scale 은 다항식 계수를 곱해 형상을 키운다. 같은 시간에 더 큰 궤적을 돌면
   속도·가속도가 그만큼 커지므로, 크기를 키울 때는 --timescale 도 같이 키울 것.
   실행 전에 실제 형상/최대속도를 출력하니 확인하고 날릴 것.

data/figure8.csv 는 crazyswarm2(crazyflie_examples, MIT) 의 것을 그대로 가져왔다.
"""
import argparse
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np

DEFAULT_SCALE = 1.0       # 궤적 크기 배율
DEFAULT_TIMESCALE = 1.5   # 시간 배율 (클수록 느림). opticalflow 는 1.5~2.0 권장
DEFAULT_HEIGHT = 1.0      # m 호버 고도
DEFAULT_LAPS = 1          # 반복 횟수


def parse_args():
    p = argparse.ArgumentParser(description='8자 다항식 궤적 비행')
    p.add_argument('--scale', type=float, default=DEFAULT_SCALE,
                   help='궤적 크기 배율 (1.0 = x ±1.0m, y ±0.54m)')
    p.add_argument('--timescale', type=float, default=DEFAULT_TIMESCALE,
                   help='시간 배율. 클수록 느림 (1.0 = 원속도 ~7.3s)')
    p.add_argument('--height', type=float, default=DEFAULT_HEIGHT,
                   help='호버 고도 [m]')
    p.add_argument('--laps', type=int, default=DEFAULT_LAPS,
                   help='궤적 반복 횟수')
    # ROS 인자(--ros-args 등)는 그대로 흘려보낸다
    args, _ = p.parse_known_args()
    return args


def scale_trajectory(traj, scale):
    """다항식 계수를 곱해 궤적 형상을 확대/축소 (yaw 는 각도라 건드리지 않음)."""
    for poly in traj.polynomials:
        poly.px.p = poly.px.p * scale
        poly.py.p = poly.py.p * scale
        poly.pz.p = poly.pz.p * scale


def traj_stats(traj, samples=400):
    """궤적의 x/y 범위와 최대 속도(궤적 시간 기준)를 계산."""
    xs, ys, vmax = [], [], 0.0
    for i in range(samples):
        t = traj.duration * i / samples
        out = traj.eval(t)
        if out is None:
            continue
        xs.append(out.pos[0])
        ys.append(out.pos[1])
        vmax = max(vmax, float(np.linalg.norm(out.vel)))
    return (min(xs), max(xs)), (min(ys), max(ys)), vmax


def main():
    args = parse_args()

    swarm = Crazyswarm()
    th = swarm.timeHelper
    allcfs = swarm.allcfs

    csv_path = Path(get_package_share_directory('crazyflie_test')) / 'data' / 'figure8.csv'
    traj = Trajectory()
    traj.loadcsv(csv_path)
    scale_trajectory(traj, args.scale)

    # 날리기 전에 실제 형상/속도 확인 (벽까지 여유 판단용)
    (xmin, xmax), (ymin, ymax), vmax = traj_stats(traj)
    lap_time = traj.duration * args.timescale
    print(f'[figure8_traj] scale={args.scale} timescale={args.timescale} '
          f'height={args.height} laps={args.laps}')
    print(f'  형상: x [{xmin:+.2f}, {xmax:+.2f}] m, y [{ymin:+.2f}, {ymax:+.2f}] m '
          f'(시작 위치 기준 상대)')
    print(f'  1바퀴 {lap_time:.1f} s, 최대 속도 ~{vmax / args.timescale:.2f} m/s')

    for cf in allcfs.crazyflies:
        cf.uploadTrajectory(0, 0, traj)

    allcfs.takeoff(targetHeight=args.height, duration=2.5)
    th.sleep(3.0)

    # 시작 위치 상공 height 로 정렬 후 궤적 재생
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0.0, 0.0, args.height])
        cf.goTo(pos, yaw=0.0, duration=2.0)
    th.sleep(2.5)

    for _ in range(args.laps):
        # relative=True (기본): 호버 지점 기준 재생. 8자는 원점에서 시작/종료
        allcfs.startTrajectory(0, timescale=args.timescale)
        th.sleep(lap_time + 1.0)

    allcfs.land(targetHeight=0.04, duration=2.5)
    th.sleep(3.0)


if __name__ == '__main__':
    main()
