"""Optical flow(Flow deck) 기반 멀티 기체 편대 비행 예제.

전 기체 동시 이륙 -> 브로드캐스트 '상대' 이동으로 정사각형 편대 비행 -> 동시 착륙.

Flow deck 은 각 기체가 자기 이륙 지점 기준으로 상대 추정을 하므로 공통 절대 좌표계가 없다.
따라서 절대 goTo 대신 allcfs.goTo(항상 relative) 브로드캐스트로 모든 기체를 같은 벡터만큼
움직여 편대를 유지한다. 드리프트 누적을 피하려고 이동 폭은 짧게 유지한다.
"""
from crazyflie_py import Crazyswarm

HEIGHT = 1.0          # m 이륙 고도
TAKEOFF_DURATION = 2.5
STEP = 0.5            # m 한 변 길이 (상대 이동, 짧게 유지)
LEG_DURATION = 3.0    # s 한 변 이동 시간
SETTLE = 1.0         # s 각 이동 후 안정화
LAND_DURATION = 2.5

# 정사각형을 그리는 상대 이동 벡터 (마지막에 시작점으로 복귀)
RELATIVE_LEGS = [
    [STEP, 0.0, 0.0],    # +x
    [0.0, STEP, 0.0],    # +y
    [-STEP, 0.0, 0.0],   # -x
    [0.0, -STEP, 0.0],   # -y
]


def main():
    swarm = Crazyswarm()
    th = swarm.timeHelper
    allcfs = swarm.allcfs

    n = len(allcfs.crazyflies)
    print(f'[multi_opticalflow] {n} 기체 편대 비행 시작')

    # 전 기체 동시 이륙 (브로드캐스트)
    allcfs.takeoff(targetHeight=HEIGHT, duration=TAKEOFF_DURATION)
    th.sleep(TAKEOFF_DURATION + SETTLE)

    # 상대 이동 브로드캐스트로 정사각형 편대 비행
    for leg in RELATIVE_LEGS:
        allcfs.goTo(leg, yaw=0.0, duration=LEG_DURATION)  # 브로드캐스트 goTo 는 항상 relative
        th.sleep(LEG_DURATION + SETTLE)

    # 전 기체 동시 착륙 (브로드캐스트)
    allcfs.land(targetHeight=0.04, duration=LAND_DURATION)
    th.sleep(LAND_DURATION + SETTLE)


if __name__ == '__main__':
    main()
