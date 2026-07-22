"""8자(리사주) 궤적 비행 — goTo 샘플링 방식 (간단·직관적).

방 중앙에서 출발하는 것을 가정하고, 8자를 원점 중심으로 그린다.
x = A*sin(wt), y = B*sin(2wt).

주의(opticalflow): goTo 는 '절대' 좌표라 Flow deck 추정 드리프트가 누적된다.
그래서 LAPS 는 1로 두고 진폭도 크게 잡지 않는다. 부드럽고 정밀한 추종이 필요하면
다항식 궤적 업로드 방식인 figure8_traj 를 쓸 것.
"""
import math

from crazyflie_py import Crazyswarm

HEIGHT = 1.0        # m
A = 1.0             # x(전진) 진폭 [m] — 방 x=11m, 중앙 출발이라 벽까지 여유 충분
B = 0.6             # y(좌우) 진폭 [m] — 방 y=8m
PERIOD = 12.0       # 한 바퀴 주기 [s]
LAPS = 1            # opticalflow 드리프트 누적 최소화를 위해 1바퀴
DT = 0.1            # 웨이포인트 샘플링 간격 [s]


def main():
    swarm = Crazyswarm()
    th = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=HEIGHT, duration=2.5)
    th.sleep(3.0)

    # 원점(방 중앙) 상공으로 정렬
    cf.goTo([0.0, 0.0, HEIGHT], yaw=0.0, duration=2.0)
    th.sleep(2.5)

    steps = int(LAPS * PERIOD / DT)
    for i in range(steps):
        t = i * DT
        omega = 2.0 * math.pi / PERIOD
        x = A * math.sin(omega * t)
        y = B * math.sin(2.0 * omega * t)
        cf.goTo([x, y, HEIGHT], yaw=0.0, duration=DT)  # yaw 고정: 회전은 flow 추정 악화
        th.sleep(DT)

    cf.goTo([0.0, 0.0, HEIGHT], yaw=0.0, duration=2.0)
    th.sleep(2.5)

    cf.land(targetHeight=0.04, duration=2.5)
    th.sleep(3.0)


if __name__ == '__main__':
    main()
