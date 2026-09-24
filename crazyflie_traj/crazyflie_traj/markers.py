"""주기 궤적을 rviz2 에 그려주는 노드.

    ros2 run crazyflie_traj traj_markers --ros-args -p shape:=clover
    rviz2 -d $(ros2 pkg prefix crazyflie_traj)/share/crazyflie_traj/config/traj.rviz

퍼블리시:
    /traj/path    nav_msgs/Path   계획된 도형 한 랩 (하늘색)
    /traj/flown   nav_msgs/Path   드론 TF(world→기체) 누적 실제 자취 (노랑)

도형은 flight 와 같은 `shapes` 로 그리므로, 파라미터(shape/scale/height/center)를
비행과 맞추면 rviz 의 경로가 실제로 나는 경로다. 도형은 이륙 지점(xy)에 얹히므로
center 를 기체의 initial_position xy 로 맞춘다(기본 0,0).
"""
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

from . import shapes as sh


def _pose(p, frame, stamp):
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.header.stamp = stamp
    ps.pose.position.x = float(p[0])
    ps.pose.position.y = float(p[1])
    ps.pose.position.z = float(p[2])
    ps.pose.orientation.w = 1.0
    return ps


class TrajMarkers(Node):

    def __init__(self):
        super().__init__('traj_markers')
        p = self.declare_parameter
        p('shape', 'clover')
        p('scale', 1.0)
        p('height', 1.0)
        p('center', [0.0, 0.0])       # 도형 xy 중심 = 기체 initial_position xy
        p('frame_id', 'world')
        p('robot_frame', 'cf231')
        p('show_trail', True)
        p('trail_step', 0.02)
        p('trail_max', 0)             # 0=무제한(끌 때까지 전부 누적)
        p('samples', 800)

        self.frame_id = self.get_parameter('frame_id').value
        self.robot_frame = self.get_parameter('robot_frame').value
        c = list(self.get_parameter('center').value)
        self.center = np.array([c[0], c[1], 0.0]) if len(c) >= 2 else np.zeros(3)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_path = self.create_publisher(Path, 'traj/path', qos)
        self.pub_flown = self.create_publisher(Path, 'traj/flown', 10)

        self._path = self._build_path()
        self.create_timer(1.0, self._publish_path)     # transient_local 이지만 재접속 대비
        self._publish_path()

        if self.get_parameter('show_trail').value:
            self.trail = []
            self.create_subscription(TFMessage, '/tf', self._on_tf, 20)
            self.create_timer(0.1, self._publish_flown)

    def _build_path(self):
        name = self.get_parameter('shape').value
        fn = sh.get_shape(name, height=self.get_parameter('height').value)
        scale = self.get_parameter('scale').value
        n = int(self.get_parameter('samples').value)
        s = np.linspace(0.0, 1.0, n)
        pos, _, _ = fn(s)
        pos = pos * np.array([scale, scale, 1.0]) + self.center
        self.get_logger().info(f'도형 {name} 경로 {n}점 (center={self.center[:2]})')
        return pos

    def _publish_path(self):
        stamp = self.get_clock().now().to_msg()
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = stamp
        msg.poses = [_pose(pt, self.frame_id, stamp) for pt in self._path]
        self.pub_path.publish(msg)

    def _on_tf(self, msg):
        step = self.get_parameter('trail_step').value
        for t in msg.transforms:
            if t.child_frame_id != self.robot_frame:
                continue
            tr = t.transform.translation
            q = (tr.x, tr.y, tr.z)
            # 끌 때까지 전부 누적: 점프로 초기화하지 않고, 이동분(step)만 쌓는다.
            if not self.trail or math.dist(q, self.trail[-1]) >= step:
                self.trail.append(q)
                cap = int(self.get_parameter('trail_max').value)
                if cap > 0 and len(self.trail) > cap:
                    self.trail.pop(0)

    def _publish_flown(self):
        stamp = self.get_clock().now().to_msg()
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = stamp
        msg.poses = [_pose(pt, self.frame_id, stamp) for pt in self.trail]
        self.pub_flown.publish(msg)


def main():
    rclpy.init()
    node = TrajMarkers()
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
