"""레이싱 정책 관측(49) 생성 + 게이트 진행 추적 — 학습 환경 racing.py 를 그대로 옮김.

관측 구성 (RacingEnv.observe, delay_gate_target_until_clear=false):
     9  다음 목표 3개까지 상대위치 × 0.5  (목표 = 게이트 1..7 중심, 마지막은 결승점)
     9  그 목표들의 통과 법선 (결승점은 0)
     8  현재 목표 번호 one-hot (0..6 = 게이트, 7 = 결승점)
     9  회전행렬 R (기체→월드), 행 우선
     3  월드 속도 / 4
     3  기체 각속도 [rad/s] / 10
     4  로터 회전수 [RPM] / 25000   (모터 m1..m4, crazyflow cf2x_L250 추력 곡선으로 환산)
     4  직전 행동
   → [-10, 10] 클립

게이트 통과는 학습과 같이 '다음 게이트 평면을 통과 방향으로 넘을 때 개구부 안'이면 센다.
"""
import numpy as np

# crazyflow cf2x_L250: 모터 추력 f = a·rpm² + b·rpm + c
RPM2THRUST = (0.0, -5.382196214637237e-7, 2.4582929831265485e-10)
N_TARGETS = 8                                      # 게이트 7 + 결승점


def force_to_rpm(force):
    c, b, a = RPM2THRUST
    f = np.maximum(np.asarray(force, dtype=float), 0.0)
    return (-b + np.sqrt(b * b - 4 * a * (c - f))) / (2 * a)


def quat_to_matrix(q):
    """xyzw 쿼터니언 → 회전행렬 (scipy Rotation.from_quat 과 같은 규약)."""
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class RacingObserver:
    """게이트 진행 상태를 들고 관측 벡터를 만든다."""

    def __init__(self, course, finish, aperture_margin=0.0):
        """course: crazyflie_racing.gate_course.Course, finish: 결승(호버) 지점 (3,).

        aperture_margin: 통과 판정 개구부 반폭에 더할 여유 [m]. 학습은 inner/2 − drone_radius
        (0.19 m)로 셌다. 실기체에서 프레임 근처로 지나 통과를 놓치면 정책이 되돌아가려 하므로
        약간 넓게 잡을 수 있다(최대 inner/2 = 0.25 m 까지가 물리적으로 통과).
        """
        gates = course.gates
        self.centres = np.array([g.center for g in gates])
        self.normals = np.array([g.normal for g in gates])
        self.laterals = np.array([g.lateral for g in gates])
        self.finish = np.asarray(finish, dtype=float)
        self.targets = np.vstack([self.centres, self.finish])
        self.target_normals = np.vstack([self.normals, np.zeros(3)])
        self.half = course.inner_size / 2 - course.drone_radius + aperture_margin
        self.gate = 0                     # 다음에 통과할 게이트 인덱스 (7 = 전부 통과)
        self.prev_pos = None

    def update(self, pos):
        """위치 갱신 → 이번 갱신에서 통과한 게이트 id(1..7), 없으면 None.

        역방향·순서 틀린 통과는 세지 않는다(학습은 실패 처리 — 여기서는 비행 쪽이 판단).
        """
        pos = np.asarray(pos, dtype=float)
        passed = None
        if self.prev_pos is not None and self.gate < len(self.centres):
            g = self.gate
            a = (self.prev_pos - self.centres[g]) @ self.normals[g]
            b = (pos - self.centres[g]) @ self.normals[g]
            if a < 0 <= b:
                frac = -a / (b - a) if abs(b - a) > 1e-8 else 0.0
                hit = self.prev_pos + frac * (pos - self.prev_pos) - self.centres[g]
                if abs(hit @ self.laterals[g]) <= self.half and abs(hit[2]) <= self.half:
                    self.gate += 1
                    passed = self.gate
        self.prev_pos = pos
        return passed

    def observe(self, pos, vel, quat_xyzw, omega_body, motor_force, prev_action):
        pos = np.asarray(pos, dtype=float)
        current = self.gate
        ids = np.minimum(current + np.arange(3), N_TARGETS - 1)
        relative = (self.targets[ids] - pos) * 0.5
        one_hot = np.zeros(N_TARGETS)
        one_hot[current] = 1.0
        obs = np.concatenate([
            relative.reshape(9), self.target_normals[ids].reshape(9), one_hot,
            quat_to_matrix(quat_xyzw).reshape(9), np.asarray(vel) / 4.0,
            np.asarray(omega_body) / 10.0, force_to_rpm(motor_force) / 25000.0,
            np.asarray(prev_action, dtype=float)])
        return np.clip(np.nan_to_num(obs), -10.0, 10.0)
