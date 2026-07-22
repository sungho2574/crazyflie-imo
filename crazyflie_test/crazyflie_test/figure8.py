"""8자 궤적 연속 비행 — cmdFullState 스트리밍 방식.

figure8_traj(다항식 업로드)는 궤적의 시작/끝 속도가 0이라, 여러 바퀴를 돌면
매 바퀴 원점에서 감속→정지→가속이 생긴다. 이 스크립트는 **주기적인 리사주 8자**를
해석적으로 계산해 setpoint 를 연속 스트리밍하므로 몇 바퀴를 돌든 멈추지 않는다.

    x = A·sin(φ),  y = B·sin(2φ),  z = height

리사주 8자는 어느 위상에서도 속도가 0이 아니라, 호버(속도 0)에서 곧바로 시작하면
속도가 튄다. 그래서 **시간 워핑(time warping)** 으로 위상 속도 φ̇ 를 0→최대→0 으로
smoothstep 램프시켜, 경로는 그대로 두면서 시작/종료만 부드럽게 만든다.
덕분에 t=0 에서 pos=중심, vel=0, acc=0 이라 호버에서 이어붙여도 충격이 없다.

    ros2 run crazyflie_test figure8_continuous --laps 3 --period 8.0 --a 1.0 --b 0.5

⚠️ cmdFullState 는 low-level 제어라 high-level commander 를 우회한다.
   - 상태추정이 튼튼해야 한다(가능하면 mocap 모드 권장, opticalflow 는 보수적으로).
   - 스트리밍이 끊기면 기체가 setpoint 를 잃으므로, 종료 시 notifySetpointsStop() 후 착륙.
"""
import argparse
import math

from crazyflie_py import Crazyswarm
import numpy as np

DEFAULT_A = 1.0        # m  x(전진) 진폭
DEFAULT_B = 0.5        # m  y(좌우) 진폭
DEFAULT_PERIOD = 8.0   # s  한 바퀴 주기 (작을수록 빠름)
DEFAULT_LAPS = 3
DEFAULT_HEIGHT = 1.0   # m
DEFAULT_RATE = 50.0    # Hz setpoint 스트리밍 주파수
DEFAULT_RAMP = 2.0     # s  시작/종료 가감속 시간


def parse_args():
    p = argparse.ArgumentParser(description='8자 연속 비행 (cmdFullState 스트리밍)')
    p.add_argument('--a', type=float, default=DEFAULT_A, help='x 진폭 [m]')
    p.add_argument('--b', type=float, default=DEFAULT_B, help='y 진폭 [m]')
    p.add_argument('--period', type=float, default=DEFAULT_PERIOD,
                   help='한 바퀴 주기 [s]. 작을수록 빠름')
    p.add_argument('--laps', type=float, default=DEFAULT_LAPS, help='바퀴 수')
    p.add_argument('--height', type=float, default=DEFAULT_HEIGHT, help='비행 고도 [m]')
    p.add_argument('--rate', type=float, default=DEFAULT_RATE, help='스트리밍 주파수 [Hz]')
    p.add_argument('--ramp', type=float, default=DEFAULT_RAMP,
                   help='시작/종료 가감속 시간 [s]')
    args, _ = p.parse_known_args()
    return args


def _smoothstep(u):
    """0->1 부드러운 램프 (u in [0,1])."""
    return u * u * (3.0 - 2.0 * u)


def _smoothstep_du(u):
    """_smoothstep 의 u 에 대한 미분."""
    return 6.0 * u * (1.0 - u)


class PhaseSchedule:
    """위상 φ(t) 스케줄: φ̇ 를 0→ω→0 으로 램프시켜 총 위상이 laps·2π 가 되게 한다."""

    def __init__(self, period, laps, ramp):
        self.omega = 2.0 * math.pi / period
        self.phi_total = laps * 2.0 * math.pi
        # 램프 구간이 전체 위상보다 크면 램프를 줄인다
        # (램프 하나가 만드는 위상 = 0.5·ω·ramp)
        max_ramp = self.phi_total / self.omega  # 두 램프 합이 전체를 넘지 않도록
        self.ramp = min(ramp, max_ramp)
        self.phi_ramp = 0.5 * self.omega * self.ramp
        self.t_cruise = max(0.0, (self.phi_total - 2.0 * self.phi_ramp) / self.omega)
        self.duration = 2.0 * self.ramp + self.t_cruise

    def eval(self, t):
        """(φ, φ̇, φ̈) 반환."""
        w, Tr = self.omega, self.ramp
        if Tr > 0.0 and t < Tr:                       # 가속 구간
            u = t / Tr
            phi = w * Tr * (u**3 - 0.5 * u**4)
            return phi, w * _smoothstep(u), w * _smoothstep_du(u) / Tr
        if t < Tr + self.t_cruise:                    # 정속 구간
            return self.phi_ramp + w * (t - Tr), w, 0.0
        if Tr > 0.0:                                  # 감속 구간
            u = min((t - Tr - self.t_cruise) / Tr, 1.0)
            v = 1.0 - u
            phi = (self.phi_ramp + w * self.t_cruise
                   + w * Tr * (0.5 - (v**3 - 0.5 * v**4)))
            return phi, w * _smoothstep(v), -w * _smoothstep_du(v) / Tr
        return self.phi_total, 0.0, 0.0


def figure8_state(phi, phid, phidd, a, b, height):
    """리사주 8자의 위치/속도/가속도 (해석적)."""
    s1, c1 = math.sin(phi), math.cos(phi)
    s2, c2 = math.sin(2.0 * phi), math.cos(2.0 * phi)
    pos = np.array([a * s1, b * s2, height])
    vel = np.array([a * phid * c1, 2.0 * b * phid * c2, 0.0])
    acc = np.array([a * (phidd * c1 - phid**2 * s1),
                    2.0 * b * (phidd * c2 - 2.0 * phid**2 * s2),
                    0.0])
    return pos, vel, acc


def preview(sched, args, samples=1000):
    """날리기 전 확인용: 형상 범위와 최대 속도/가속도."""
    vmax = amax = 0.0
    for i in range(samples + 1):
        t = sched.duration * i / samples
        phi, phid, phidd = sched.eval(t)
        _, vel, acc = figure8_state(phi, phid, phidd, args.a, args.b, args.height)
        vmax = max(vmax, float(np.linalg.norm(vel)))
        amax = max(amax, float(np.linalg.norm(acc)))
    return vmax, amax


def main():
    args = parse_args()
    sched = PhaseSchedule(args.period, args.laps, args.ramp)
    vmax, amax = preview(sched, args)

    print(f'[figure8_continuous] a={args.a} b={args.b} period={args.period} '
          f'laps={args.laps} height={args.height} rate={args.rate}')
    print(f'  형상: x ±{args.a:.2f} m, y ±{args.b:.2f} m (시작 위치 중심)')
    print(f'  총 {sched.duration:.1f} s (가감속 {sched.ramp:.1f} s ×2), '
          f'최대 속도 ~{vmax:.2f} m/s, 최대 가속도 ~{amax:.2f} m/s^2')

    swarm = Crazyswarm()
    th = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    center = np.array(cf.initialPosition) + np.array([0.0, 0.0, args.height])

    cf.takeoff(targetHeight=args.height, duration=2.5)
    th.sleep(3.0)

    # 8자 중심(=시작 위치 상공)으로 정렬. φ=0 에서 궤적도 정확히 이 점이다.
    cf.goTo(center, yaw=0.0, duration=2.0)
    th.sleep(2.5)

    start = th.time()
    while not th.isShutdown():
        t = th.time() - start
        if t > sched.duration:
            break
        phi, phid, phidd = sched.eval(t)
        pos, vel, acc = figure8_state(phi, phid, phidd, args.a, args.b, args.height)
        # height 는 pos 에 이미 들어있으므로 중심의 xy 만 더한다
        cf.cmdFullState(center + np.array([pos[0], pos[1], 0.0]),
                        vel, acc, 0.0, np.zeros(3))
        th.sleepForRate(args.rate)

    cf.notifySetpointsStop()
    cf.goTo(center, yaw=0.0, duration=2.0)
    th.sleep(2.5)

    cf.land(targetHeight=0.04, duration=2.5)
    th.sleep(3.0)


if __name__ == '__main__':
    main()
