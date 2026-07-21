"""8자(리사주) 궤적 비행 — 연속 궤적 추종 성능 확인용."""
import math

from crazyflie_py import Crazyswarm

HEIGHT = 1.0
A = 0.5             # x 진폭 [m]
B = 0.3             # y 진폭 [m]
PERIOD = 12.0       # 한 바퀴 주기 [s]
LAPS = 2
DT = 0.1            # 웨이포인트 샘플링 간격 [s]


def main():
    swarm = Crazyswarm()
    th = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=HEIGHT, duration=2.5)
    th.sleep(3.0)

    # 시작점으로 이동
    cf.goTo([0.0, 0.0, HEIGHT], yaw=0.0, duration=2.0)
    th.sleep(2.5)

    steps = int(LAPS * PERIOD / DT)
    for i in range(steps):
        t = i * DT
        omega = 2.0 * math.pi / PERIOD
        x = A * math.sin(omega * t)
        y = B * math.sin(2.0 * omega * t)
        cf.goTo([x, y, HEIGHT], yaw=0.0, duration=DT)
        th.sleep(DT)

    cf.goTo([0.0, 0.0, HEIGHT], yaw=0.0, duration=2.0)
    th.sleep(2.5)

    cf.land(targetHeight=0.04, duration=2.5)
    th.sleep(3.0)


if __name__ == '__main__':
    main()
