"""8자 궤적 비행 — 다항식 궤적 업로드 방식 (정석·부드러움).

crazyswarm2 표준 figure8.csv(10구간 조각다항식, ~7.3s)를 기체에 업로드해
high-level commander 가 연속 궤적을 추종한다. goTo 샘플링보다 훨씬 부드럽고,
mocap 으로 넘어가도 그대로 쓸 수 있다.

궤적 형상은 고정: 원점 중심 약 x 2.0m × y 1.0m (z=0, 시작 위치 기준 상대 재생).
방 중앙(x=11m, y=8m)에서 1m 고도로 호버 후 재생하므로 벽까지 여유는 충분하다.
속도는 TIMESCALE 로 조절한다 (클수록 느림 → opticalflow 에서 안정적).

data/figure8.csv 는 crazyswarm2(crazyflie_examples, MIT) 의 것을 그대로 가져왔다.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np

HEIGHT = 1.0        # m 호버 고도 (궤적은 이 위치 기준 상대 재생)
TIMESCALE = 1.5     # 1.0 = 원속도(~7.3s), 클수록 느림. opticalflow 는 1.5~2.0 권장


def main():
    swarm = Crazyswarm()
    th = swarm.timeHelper
    allcfs = swarm.allcfs

    csv_path = Path(get_package_share_directory('crazyflie_test')) / 'data' / 'figure8.csv'
    traj = Trajectory()
    traj.loadcsv(csv_path)

    for cf in allcfs.crazyflies:
        cf.uploadTrajectory(0, 0, traj)

    allcfs.takeoff(targetHeight=HEIGHT, duration=2.5)
    th.sleep(3.0)

    # 시작 위치(방 중앙) 상공 HEIGHT 로 정렬 후 궤적 재생
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0.0, 0.0, HEIGHT])
        cf.goTo(pos, yaw=0.0, duration=2.0)
    th.sleep(2.5)

    allcfs.startTrajectory(0, timescale=TIMESCALE)   # relative=True (기본): 호버 지점 기준 재생
    th.sleep(traj.duration * TIMESCALE + 1.0)

    allcfs.land(targetHeight=0.04, duration=2.5)
    th.sleep(3.0)


if __name__ == '__main__':
    main()
