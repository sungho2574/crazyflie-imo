"""게이트 코스 정의 · 기하 · 검증 (ROS 의존 없음).

`config/gates.yaml` 을 읽어 게이트 배치와 방·이착륙 지점을 객체로 만든다. 실제
비행 경로는 **TOGT-Planner 로 오프라인 생성**한 다항식 궤적 CSV
(`config/gate_trajectory.csv`)이고, 그 궤적을 크플 펌웨어의 High-Level Commander
가 실행한다(`gate_flight`). 이 모듈은 그 궤적을 **검사·시각화**하는 데 쓰이는
기하 라이브러리다.

    gates.yaml ──► (오프라인: plan_gate_trajectory + TOGT) ──► gate_trajectory.csv
                                                                     │
                          gate_flight (펌웨어 uploadTrajectory) ◄─────┤
                          gate_markers (rviz Path)            ◄─────┘

좌표계
    gates.yaml 의 x, y 는 **world 프레임 절대 좌표**이고 방 중심이 원점이다.
    (size_x=8 → x ∈ [-4, 4], size_y=11 → y ∈ [-5.5, 5.5])
    기체의 `initial_position`(crazyflies_*.yaml)이 gates.yaml 의 `start` 와 같아야
    게이트 좌표가 맞는다 (gate_flight 가 실행 전에 검사한다).

게이트 형상
    mount_z 는 게이트 **하단** 높이라, 통과 고도(개구부 중심)는
    mount_z + inner_size/2 = mount_z + 0.25 m 다.
    yaw_deg 는 통과 방향(법선)의 방위각. **위에서 봤을 때 시계방향(CW)** 이므로
    진행 방향 = (cos(−yaw_deg), sin(−yaw_deg)) 다. 이 규약의 출처는 게이트 맵
    생성기 gate_map.py 의 `yaw_rad()`. `Gate.yaw` 참고.
"""
import math
import os

import numpy as np
import yaml

DEFAULT_CLEARANCE = 0.12  # m   게이트 프레임에서 띄울 최소 거리 (검사 기준)
MIN_ALTITUDE = 0.15       # m   이보다 낮게 날면 지면효과 경고 (gate_map.py 와 동일)
MIN_SELF_GAP = 0.4        # m   경로가 자기 자신과 가까워질 때 경고 기준


# --------------------------------------------------------------------------- #
# 코스 정의
# --------------------------------------------------------------------------- #
class Gate:
    """게이트 한 개. 중심 좌표와 통과 방향(법선)을 제공한다."""

    def __init__(self, gid, x, y, mount_z, yaw_deg, inner_size, frame_thickness):
        self.id = gid
        self.x = float(x)
        self.y = float(y)
        self.mount_z = float(mount_z)
        self.yaw_deg = float(yaw_deg)
        self.inner_size = float(inner_size)
        self.frame_thickness = float(frame_thickness)

    @property
    def yaw(self):
        """통과 방향 [rad].

        ⚠️ yaw_deg 는 **위에서 봤을 때 시계방향(CW)** 기준이다. 수학 표준
        (atan2 = CCW)과 부호가 반대라 여기서 뒤집는다. 이 규약의 출처는
        게이트 맵 생성기 gate_map.py 의 `yaw_rad()` — 실제 게이트를 그
        도면대로 놓으므로 그쪽이 기준이다.
        (gates.yaml 헤더 주석에는 'CCW' 라고 적혀 있는데 그게 틀렸다.)
        """
        return math.radians(-self.yaw_deg)

    @property
    def center(self):
        """개구부 중심 = 통과 고도."""
        return np.array([self.x, self.y, self.mount_z + self.inner_size / 2.0])

    @property
    def normal(self):
        """통과 방향 단위벡터 (수평)."""
        return np.array([math.cos(self.yaw), math.sin(self.yaw), 0.0])

    @property
    def lateral(self):
        """개구부 평면의 가로 방향 단위벡터."""
        return np.array([-math.sin(self.yaw), math.cos(self.yaw), 0.0])

    def __repr__(self):
        c = self.center
        return (f'Gate({self.id}: xyz=({c[0]:.2f}, {c[1]:.2f}, {c[2]:.2f}), '
                f'yaw={self.yaw_deg:.0f}deg)')


class Room:
    """비행 공간. 방 중심이 원점이라고 가정한다."""

    def __init__(self, size_x, size_y, height, safety_margin):
        self.size_x = float(size_x)
        self.size_y = float(size_y)
        self.height = float(height)
        self.safety_margin = float(safety_margin)

    def bounds(self):
        """방 실제 경계 (min, max)."""
        return (np.array([-self.size_x / 2, -self.size_y / 2, 0.0]),
                np.array([self.size_x / 2, self.size_y / 2, self.height]))

    def safe_bounds(self):
        """안전 여유를 뺀 권장 비행 영역.

        safety_margin 은 **수평에만** 적용한다. 천장까지 1.5 m 를 빼면
        게이트 상단(mount_z 1.0 + 0.5 = 1.5 m)보다 낮아져 성립하지 않는다.
        """
        m = self.safety_margin
        return (np.array([-self.size_x / 2 + m, -self.size_y / 2 + m, 0.0]),
                np.array([self.size_x / 2 - m, self.size_y / 2 - m, self.height]))


class Course:
    """gates.yaml 한 벌."""

    def __init__(self, room, gates, inner_size, frame_thickness, allowed_mount_z,
                 start=None, takeoff_z=None, drone_radius=0.06, frame_margin=0.10):
        self.room = room
        self.gates = gates                      # id 순 정렬된 list[Gate]
        self.inner_size = inner_size
        self.frame_thickness = frame_thickness
        self.allowed_mount_z = allowed_mount_z
        # 실기체 충돌 방지: 궤적(기체 중심)을 게이트 중앙 쪽으로 좁혀 통과시킨다.
        self.drone_radius = float(drone_radius)
        self.frame_margin = float(frame_margin)
        # 이륙/착륙 지점 (바닥). yaml 에 없으면 첫 게이트 앞쪽에 잡는다.
        if start is None:
            g = gates[0]
            start = g.center - 1.1 * g.normal
        self.start = np.array([start[0], start[1], 0.0])
        # 이륙 후 호버 고도 — 기본은 첫 게이트 통과 고도
        self.takeoff_z = float(takeoff_z if takeoff_z is not None
                               else gates[0].center[2])

    def start_hover(self, height=None):
        """이륙 후 호버 지점 (= 착륙 직전 지점). 이륙 지점과 착륙 지점은 같다."""
        return np.array([self.start[0], self.start[1],
                         self.takeoff_z if height is None else float(height)])

    @property
    def traversal_half(self):
        """궤적(기체 중심)이 지나야 하는 게이트 유효 창의 반폭 [m].

        inner/2 − (drone_radius + frame_margin). 이보다 크게 벗어나면 프로펠러가
        프레임에 frame_margin 미만으로 다가간다. TOGT marginW/H = inner − 2·이 값.
        """
        h = self.inner_size / 2.0 - (self.drone_radius + self.frame_margin)
        return max(h, 0.02)     # 게이트보다 여유가 커도 최소한의 창은 남긴다

    def by_id(self, gid):
        for g in self.gates:
            if g.id == gid:
                return g
        raise KeyError(f'게이트 id {gid} 가 gates.yaml 에 없다')


def default_gates_path():
    """설치된 share 디렉터리 → 소스 트리 순으로 gates.yaml 을 찾는다."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(
            get_package_share_directory('crazyflie_racing'), 'config', 'gates.yaml')
        if os.path.exists(path):
            return path
    except Exception:       # ROS 환경이 아니어도 (오프라인 계획 등) 동작하게
        pass
    return os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config', 'gates.yaml')


def load_course(path=None):
    path = path or default_gates_path()
    with open(path, 'r') as f:
        data = yaml.safe_load(f)

    r = data['room']
    room = Room(r['size_x'], r['size_y'], r['height'], r.get('safety_margin', 0.0))

    gspec = data.get('gate', {})
    inner = float(gspec.get('inner_size', 0.5))
    thick = float(gspec.get('frame_thickness', 0.08))

    gates = [Gate(g['id'], g['x'], g['y'], g['mount_z'], g['yaw_deg'], inner, thick)
             for g in data['gates']]
    gates.sort(key=lambda g: g.id)
    if not gates:
        raise ValueError(f'{path}: gates 가 비어 있다')

    s = data.get('start') or {}
    start = [s['x'], s['y']] if 'x' in s and 'y' in s else None

    return Course(room, gates, inner, thick, gspec.get('allowed_mount_z', []),
                  start=start, takeoff_z=s.get('takeoff_z'),
                  drone_radius=gspec.get('drone_radius', 0.06),
                  frame_margin=gspec.get('frame_margin', 0.10))


# --------------------------------------------------------------------------- #
# 궤적 로딩 · 샘플링
# --------------------------------------------------------------------------- #
def _config_path(name):
    """설치된 share → 소스 트리 순으로 config/<name> 을 찾는다."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('crazyflie_racing'), 'config', name)
        if os.path.exists(path):
            return path
    except Exception:
        pass
    return os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config', name)


def default_trajectory_path():
    return _config_path('gate_trajectory.csv')


# 연속 여러 바퀴(--loop): 진입(start 호버→이음매) · 순환 1랩(이음매→이음매) · 탈출(이음매→start 호버)
LOOP_PARTS = ('entry', 'lap', 'exit')


def default_loop_paths():
    return {part: _config_path(f'gate_loop_{part}.csv') for part in LOOP_PARTS}


def load_rows(path):
    """궤적 CSV → (조각 수, 33) 배열."""
    return np.atleast_2d(np.loadtxt(path, delimiter=',', skiprows=1, usecols=range(33)))


def _eval_row(row, t):
    """조각 하나의 t 에서 (pos, vel, acc) — 각 (3,)."""
    out = []
    for k in range(3):
        c = np.polynomial.polynomial.Polynomial(row[1 + 8 * k:9 + 8 * k])
        out.append([c(t), c.deriv(1)(t), c.deriv(2)(t)])
    return np.array(out).T


def seam_error(rows_a, rows_b):
    """궤적 a 의 끝과 b 의 시작 사이 (pos, vel, acc) 최대 차이. 이어 붙일 수 있는지 검사용."""
    end = _eval_row(rows_a[-1], rows_a[-1][0])
    start = _eval_row(rows_b[0], 0.0)
    return np.abs(end - start).max(axis=1)


def load_trajectory(path=None):
    """크플 다항식 궤적 CSV → crazyflie_py 의 Trajectory 객체.

    ROS(crazyflie_py) 가 없으면 ImportError 가 난다. 그런 환경에서는
    `sample_trajectory` 가 crazyflie_py 없이도 동작한다.
    """
    from crazyflie_py.uav_trajectory import Polynomial4D, Trajectory
    # Trajectory.loadcsv 는 조각이 1개인 CSV 를 1차원으로 읽어 깨진다(gate_loop_entry.csv).
    # 항상 2차원으로 읽는 load_rows 로 직접 채운다.
    rows = load_rows(path or default_trajectory_path())
    traj = Trajectory()
    traj.polynomials = [Polynomial4D(r[0], r[1:9], r[9:17], r[17:25], r[25:33])
                        for r in rows]
    traj.duration = float(rows[:, 0].sum())
    return traj


def sample_trajectory(path=None, n=4000):
    """다항식 궤적 CSV 를 (N,3) 점열로 샘플링. crazyflie_py 없이 동작.

    CSV = duration, x^0..x^7, y^0..y^7, z^0..z^7, yaw^0..yaw^7 (오름차순, 조각별 절대초).
    """
    return sample_rows(load_rows(path or default_trajectory_path()), n)


def sample_rows(rows, n=4000):
    """조각 배열(load_rows) → (N,3) 점열. 여러 궤적을 이어 붙인 배열도 된다."""
    per = max(2, int(n / len(rows)))
    pts = []
    for r in rows:
        dur = r[0]
        cx, cy, cz = r[1:9][::-1], r[9:17][::-1], r[17:25][::-1]   # np.polyval 은 내림차순
        for k in range(per):
            t = dur * k / per
            pts.append([np.polyval(cx, t), np.polyval(cy, t), np.polyval(cz, t)])
    return np.array(pts)


# --------------------------------------------------------------------------- #
# 게이트 프레임 간섭 검사
# --------------------------------------------------------------------------- #
def frame_conflicts(points, course, clearance=DEFAULT_CLEARANCE):
    """경로 점열이 게이트 프레임에 걸리는 지점을 찾는다.

    게이트 개구부 평면을 지나는 곳을 모두 찾아, 프레임(안쪽 테두리)에서
    clearance 이상 떨어져 지나가는지 본다. 평면을 프레임 바깥에서 지나면 무시.
    반환: [{gate, margin, point}] — margin<0 이면 프레임 관통.
    """
    points = np.asarray(points)
    half = course.inner_size / 2.0
    outer = half + course.frame_thickness
    out = []
    for g in course.gates:
        c, n, lat = g.center, g.normal, g.lateral
        sd = (points - c) @ n
        for i in np.where(np.sign(sd[:-1]) != np.sign(sd[1:]))[0]:
            denom = sd[i] - sd[i + 1]
            w = sd[i] / denom if abs(denom) > 1e-12 else 0.0
            p = points[i] + w * (points[i + 1] - points[i])
            du = abs(float(np.dot(p - c, lat)))
            dv = abs(float(p[2] - c[2]))
            if max(du, dv) > outer:
                continue                       # 프레임 바깥으로 비켜 지나감
            margin = half - max(du, dv)
            if margin < clearance:
                out.append({'gate': g, 'margin': margin, 'point': p})
    return out


def gate_edge_clearance(points, course):
    """게이트별 **프로펠러 끝 ~ 프레임 최소 여유** [m]. {gate_id: margin}.

    궤적(기체 중심)이 개구부 평면을 지나는 지점에서, 프레임 안쪽 테두리까지의
    거리(중심 여유)에서 drone_radius 를 뺀 값. 음수면 프로펠러가 프레임에 닿는다.
    """
    points = np.asarray(points)
    half = course.inner_size / 2.0
    out = {}
    for g in course.gates:
        c, n, lat = g.center, g.normal, g.lateral
        sd = (points - c) @ n
        for i in np.where(np.sign(sd[:-1]) != np.sign(sd[1:]))[0]:
            denom = sd[i] - sd[i + 1]
            w = sd[i] / denom if abs(denom) > 1e-12 else 0.0
            p = points[i] + w * (points[i + 1] - points[i])
            du = abs(float(np.dot(p - c, lat)))
            dv = abs(float(p[2] - c[2]))
            if max(du, dv) > half + course.frame_thickness:
                continue                        # 프레임 바깥으로 비켜 지나감
            edge = (half - max(du, dv)) - course.drone_radius
            if g.id not in out or edge < out[g.id]:
                out[g.id] = edge
    return out


def check_gate_clearance(points, course):
    """게이트 프레임을 스치거나 뚫는 곳을 사람이 읽는 문자열로. (없으면 [])"""
    hits = []
    for b in frame_conflicts(points, course, clearance=0.05):
        g, m, p = b['gate'], b['margin'], b['point']
        du = abs(float(np.dot(p - g.center, g.lateral)))
        dv = abs(float(p[2] - g.center[2]))
        if m < 0.0:
            hits.append(f'게이트 {g.id} 프레임과 충돌 (중심에서 '
                        f'가로 {du:.2f} m, 세로 {dv:.2f} m)')
        else:
            hits.append(f'게이트 {g.id} 개구부 가장자리 통과 (여유 {m:.3f} m)')
    return hits


# --------------------------------------------------------------------------- #
# 사전 점검 (궤적 점열 기준)
# --------------------------------------------------------------------------- #
def preview(points, course, home_xy=None):
    """gate_map.py 의 [check] 항목과 같은 것들을 궤적 점열에서 본다.

    bbox, 벽까지 여유, 최저 고도, 경로 자기 이격, 방 경계 이탈.
    """
    pts = np.asarray(points)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    room = course.room

    # 경로가 자기 자신과 가까워지는 최소 이격 (경로를 따라 충분히 떨어진 쌍만)
    thin = pts[::max(1, len(pts) // 400)]
    n = len(thin)
    d = np.linalg.norm(thin[:, None, :] - thin[None, :, :], axis=-1)
    idx = np.abs(np.arange(n)[:, None] - np.arange(n)[None, :])
    far = idx > n // 12
    hxy = np.asarray(home_xy if home_xy is not None else pts[0, :2])
    home = np.linalg.norm(thin[:, :2] - hxy, axis=1) < 0.5   # 이착륙 상공은 제외
    far &= ~home[:, None] & ~home[None, :]
    self_gap = float(d[far].min()) if far.any() else float('inf')

    rlo, rhi = room.bounds()
    wall = float(min((lo[:2] - rlo[:2]).min(), (rhi[:2] - hi[:2]).min()))
    escaped = int(((pts[:, 0] < rlo[0]) | (pts[:, 0] > rhi[0]) |
                   (pts[:, 1] < rlo[1]) | (pts[:, 1] > rhi[1]) |
                   (pts[:, 2] < 0.0) | (pts[:, 2] > room.height)).sum())

    warnings = []
    if escaped:
        warnings.append(f'경로가 방 경계를 {escaped} 점 벗어난다')
    elif wall < room.safety_margin:
        warnings.append(
            f'벽까지 여유 {wall:.2f} m < 안전여유 기준 {room.safety_margin:.2f} m. '
            f'게이트가 여유선 위에 놓여 있어 진입/탈출 구간은 넘을 수밖에 없다 '
            f'(방 자체는 안 넘음)')
    if lo[2] < MIN_ALTITUDE:
        warnings.append(f'최저 고도 {lo[2]:.2f} m — 지면효과 주의')
    if self_gap < MIN_SELF_GAP:
        warnings.append(
            f'경로가 자기 자신과 {self_gap:.2f} m 까지 붙는다(기준 '
            f'{MIN_SELF_GAP:.2f} m). 한 대만 날리면 시간차로 통과하므로 충돌은 '
            f'아니지만, 같은 통로를 두 번 쓴다는 뜻이라 위치 오차가 크면 주의. '
            f'게이트 배치에서 오는 것이라 gate_map.py 기준 경로도 같은 곳에서 교차')
    return {
        'bbox': (lo, hi),
        'wall': wall,
        'self_gap': self_gap,
        'escaped': escaped,
        'warnings': warnings,
    }
