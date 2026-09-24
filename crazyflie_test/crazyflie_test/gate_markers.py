"""게이트 코스를 rviz2 에 그려주는 노드.

    ros2 run crazyflie_test gate_markers
    rviz2 -d $(ros2 pkg prefix crazyflie_test)/share/crazyflie_test/config/gate_course.rviz

퍼블리시하는 토픽
    /gate_course/markers   visualization_msgs/MarkerArray
                           방 경계 · 게이트 프레임(초록) · 유효 통과 창(주황) ·
                           게이트 번호 · 통과 방향 화살표 · 이·착륙 지점
    /gate_course/path      nav_msgs/Path   **계획** 궤적 (config/gate_trajectory.csv)
    /gate_course/flown     nav_msgs/Path   **실제 비행 자취** (TF world→기체 누적)

계획 경로는 `gate_flight` 가 실행하는 **같은 궤적 CSV** 를 샘플해 그린다. 실제 비행
자취는 드론 TF 를 받아 누적해 그리므로, 둘을 겹쳐 계획 대비 실제 추종을 볼 수 있다.
다른 궤적을 그리려면 -p trajectory:=..., 기체 이름이 다르면 -p robot_frame:=... .

QoS 는 transient_local 이라 rviz 를 나중에 켜도 마커가 뜬다. 그래도 재접속에
대비해 1 Hz 로 다시 쏜다.
"""
import math

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from . import gate_course as gc

ROOM_COLOR = (0.55, 0.60, 0.70, 0.9)
SAFE_COLOR = (1.00, 0.65, 0.10, 0.8)
HOME_COLOR = (0.40, 0.85, 1.00, 0.9)
GATE_COLOR = (0.20, 0.80, 0.25, 1.0)    # 게이트 프레임은 전부 초록 통일
WINDOW_COLOR = (1.00, 0.75, 0.10, 0.95)  # TOGT 목표 마진(유효 통과 창)


def _color(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def _point(p):
    return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))


class GateMarkers(Node):

    def __init__(self):
        super().__init__('gate_markers')
        p = self.declare_parameter
        p('gates_yaml', '')         # 비우면 패키지 config/gates.yaml
        p('trajectory', '')         # 비우면 패키지 config/gate_trajectory.csv
        p('frame_id', 'world')
        p('start', [])              # 이륙/착륙 xy. 비우면 gates.yaml 의 start 를 쓴다
        p('height', 0.0)            # 이륙 고도. 0 이면 gates.yaml 의 start.takeoff_z
        p('show_path', True)
        p('rate', 1.0)
        # 실제 비행 자취(trail)
        p('show_trail', True)       # 드론이 지나간 실제 경로를 그린다
        p('robot_frame', 'cf231')   # TF child_frame_id (기체 이름). world→이 프레임
        p('trail_step', 0.02)       # 이만큼 움직였을 때만 점 추가 [m]
        p('trail_max', 0)           # 자취 최대 점 수. 0=무제한(끌 때까지 전부 누적)

        self.frame_id = self.get_parameter('frame_id').value
        self.course = gc.load_course(self.get_parameter('gates_yaml').value or None)
        start = list(self.get_parameter('start').value)
        if len(start) >= 2:
            self.course.start = np.array([start[0], start[1], 0.0])
        self.samples = self._load_trajectory()

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_markers = self.create_publisher(
            MarkerArray, 'gate_course/markers', qos)
        self.pub_path = self.create_publisher(Path, 'gate_course/path', qos)

        # 코스·궤적은 정적이라 한 번만 만들고, 이후엔 timestamp 만 갱신해 다시 쏜다
        self._markers = self.build_markers()
        self._path = self.build_path() if self.samples is not None and \
            self.get_parameter('show_path').value else None

        self.create_timer(1.0 / max(0.1, self.get_parameter('rate').value),
                          self.publish)
        self.publish()

        # 실제 비행 자취: TF(world→robot_frame)를 받아 누적 Path 로 퍼블리시.
        if self.get_parameter('show_trail').value:
            self.robot_frame = self.get_parameter('robot_frame').value
            self.trail = []                 # [(x,y,z), ...]
            self.pub_trail = self.create_publisher(Path, 'gate_course/flown', 10)
            self.create_subscription(TFMessage, '/tf', self._on_tf, 20)
            self.create_timer(0.1, self._publish_trail)   # 10 Hz

    def _on_tf(self, msg):
        step = self.get_parameter('trail_step').value
        for t in msg.transforms:
            if t.child_frame_id != self.robot_frame:
                continue
            tr = t.transform.translation
            p = (tr.x, tr.y, tr.z)
            # 끌 때까지 전부 누적: 점프로 초기화하지 않고, 이동분(step)만 쌓는다.
            if not self.trail or math.dist(p, self.trail[-1]) >= step:
                self.trail.append(p)
                cap = int(self.get_parameter('trail_max').value)
                if cap > 0 and len(self.trail) > cap:
                    self.trail.pop(0)

    def _publish_trail(self):
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for p in self.trail:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position = _point(p)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_trail.publish(msg)

    def _load_trajectory(self):
        """gate_flight 가 실제로 날 궤적 CSV 를 점열로 로드. 없으면 None(게이트만 그림)."""
        path = self.get_parameter('trajectory').value or None
        try:
            samples = gc.sample_trajectory(path)
            self.get_logger().info(
                f'궤적 로드: {path or gc.default_trajectory_path()} '
                f'({len(samples)} 점)')
            hits = gc.check_gate_clearance(samples, self.course)
            for h in hits:
                self.get_logger().warning(h)
            return samples
        except Exception as exc:
            self.get_logger().warning(
                f'궤적 CSV 를 읽지 못했다({exc}). 게이트만 그린다. '
                'plan_gate_trajectory 로 먼저 생성할 것')
            return None

    # ------------------------------------------------------------------ #
    def _marker(self, ns, mid, mtype, scale, color):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = [float(v) for v in scale]
        m.color = color if isinstance(color, ColorRGBA) else _color(*color)
        return m

    def _box_edges(self, lo, hi):
        c = [(x, y, z) for x in (lo[0], hi[0])
             for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
        edges = [(0, 1), (2, 3), (4, 5), (6, 7),        # z 방향
                 (0, 2), (1, 3), (4, 6), (5, 7),        # y 방향
                 (0, 4), (1, 5), (2, 6), (3, 7)]        # x 방향
        pts = []
        for a, b in edges:
            pts += [_point(c[a]), _point(c[b])]
        return pts

    def _rect(self, lo, hi, z):
        c = [(lo[0], lo[1], z), (hi[0], lo[1], z), (hi[0], hi[1], z), (lo[0], hi[1], z)]
        pts = []
        for i in range(4):
            pts += [_point(c[i]), _point(c[(i + 1) % 4])]
        return pts

    def _plane_rect(self, gate, half):
        """게이트 평면 안의 정사각형(반폭 half). 로컬 가로=lateral, 세로=z."""
        lat = gate.lateral
        corners = [gate.center + su * half * lat + np.array([0, 0, sv * half])
                   for su, sv in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
        pts = []
        for i in range(4):
            pts += [_point(corners[i]), _point(corners[(i + 1) % 4])]
        return pts

    def build_markers(self):
        room = self.course.room
        arr = MarkerArray()

        lo, hi = room.bounds()
        m = self._marker('room', 0, Marker.LINE_LIST, (0.02, 0, 0), ROOM_COLOR)
        m.points = self._box_edges(lo, hi)
        arr.markers.append(m)

        slo, shi = room.safe_bounds()
        m = self._marker('safe_area', 0, Marker.LINE_LIST, (0.03, 0, 0), SAFE_COLOR)
        # safety_margin 은 수평 여유라 바닥에만 그린다 (gate_course.Room 참고)
        m.points = self._rect(slo, shi, 0.01)
        arr.markers.append(m)

        # 게이트는 전부 초록으로 통일. 통과 순서는 라벨(G1..G7)로 안다.
        color = _color(*GATE_COLOR)
        th = self.course.traversal_half     # TOGT 목표 마진(유효 창) 반폭
        for g in self.course.gates:
            for j, m in enumerate(self._gate_frame(g, color)):
                m.ns, m.id = 'gate_frame', g.id * 10 + j
                arr.markers.append(m)

            # TOGT 가 궤적(기체 중심)을 통과시키는 유효 창(주황 사각형).
            # 프레임 안쪽으로 (drone_radius + frame_margin) 만큼 좁혀져 있다.
            m = self._marker('gate_window', g.id, Marker.LINE_LIST,
                             (0.012, 0, 0), WINDOW_COLOR)
            m.points = self._plane_rect(g, th)
            arr.markers.append(m)

            top = g.center[2] + g.inner_size / 2 + g.frame_thickness + 0.18
            t = self._marker('gate_label', g.id, Marker.TEXT_VIEW_FACING,
                             (0, 0, 0.22), color)
            t.text = f'G{g.id}'     # 통과 순서는 프레임 색(초록→빨강)으로 보여준다
            t.pose.position = _point([g.x, g.y, top])
            arr.markers.append(t)

            a = self._marker('gate_dir', g.id, Marker.ARROW,
                             (0.03, 0.07, 0.09), color)
            a.points = [_point(g.center - 0.1 * g.normal),
                        _point(g.center + 0.45 * g.normal)]
            arr.markers.append(a)

        # 이륙 지점과 착륙 지점은 **같은 곳**이다 (gates.yaml 의 start).
        # 바닥 원판 + 호버 고도까지의 수직선 + 글씨로 표시한다.
        s = self.course.start
        hover = self.course.start_hover(self.get_parameter('height').value or None)
        m = self._marker('home', 0, Marker.CYLINDER, (0.36, 0.36, 0.02), HOME_COLOR)
        m.pose.position = _point([s[0], s[1], 0.01])
        arr.markers.append(m)

        m = self._marker('home', 1, Marker.LINE_LIST, (0.012, 0, 0), HOME_COLOR)
        m.points = [_point([s[0], s[1], 0.0]), _point(hover)]
        arr.markers.append(m)

        m = self._marker('home', 2, Marker.SPHERE, (0.1, 0.1, 0.1), HOME_COLOR)
        m.pose.position = _point(hover)
        arr.markers.append(m)

        m = self._marker('home', 3, Marker.TEXT_VIEW_FACING, (0, 0, 0.2), HOME_COLOR)
        m.text = 'TAKEOFF'      # 착륙 지점도 같은 곳이지만 표시는 TAKEOFF 로만
        m.pose.position = _point([s[0], s[1], hover[2] + 0.22])
        arr.markers.append(m)
        return arr

    def _gate_frame(self, gate, color):
        """게이트 프레임을 막대 4개(CUBE)로. 로컬 x=법선, y=가로, z=위."""
        half = gate.inner_size / 2.0
        t = gate.frame_thickness
        off = half + t / 2.0
        q = math.sin(gate.yaw / 2.0), math.cos(gate.yaw / 2.0)
        bars = [(gate.center + off * gate.lateral, (t, t, gate.inner_size + 2 * t)),
                (gate.center - off * gate.lateral, (t, t, gate.inner_size + 2 * t)),
                (gate.center + np.array([0, 0, off]), (t, gate.inner_size + 2 * t, t)),
                (gate.center - np.array([0, 0, off]), (t, gate.inner_size + 2 * t, t))]
        out = []
        for pos, scale in bars:
            m = self._marker('gate_frame', 0, Marker.CUBE, scale, color)
            m.pose.position = _point(pos)
            m.pose.orientation.z, m.pose.orientation.w = float(q[0]), float(q[1])
            out.append(m)
        return out

    def build_path(self):
        """rviz Path = gate_flight 가 실제로 날 궤적 CSV 를 샘플한 것."""
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for p in self.samples:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position = _point(p)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg

    def publish(self):
        now = self.get_clock().now().to_msg()
        for m in self._markers.markers:
            m.header.stamp = now
        self.pub_markers.publish(self._markers)
        if self._path is not None:
            self._path.header.stamp = now
            self.pub_path.publish(self._path)


def main():
    rclpy.init()
    node = GateMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
