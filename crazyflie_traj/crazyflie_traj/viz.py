"""비행 중 rviz 시각화 — flight.run 이 직접 붙인다 (별도 노드·launch 불필요).

    /traj/path    nav_msgs/Path   이번 비행의 계획 도형 한 랩 (하늘색)
    /traj/flown   nav_msgs/Path   이번 비행의 실제 자취, 드론 TF 누적 (노랑)

계획선은 비행과 **같은 도형 함수·오프셋**으로 그리므로 shape/scale/height 를 따로 맞출
필요가 없다. 퍼블리셔는 crazyflie_py 의 노드(swarm.allcfs)에 얹는다 — TimeHelper 의
sleep/sleepForRate 가 이 노드를 spin 하므로 타이머·구독이 비행 루프 중에도 돈다.
rviz 는 이미 traj.rviz 로 떠 있으면 새로 띄우지 않고, 비행이 끝나도 닫지 않는다
(마지막 계획선·자취가 남아 있어 비행 후 비교할 수 있다).
"""
import math
import os
import subprocess

import numpy as np

RVIZ_CONFIG = 'traj.rviz'
TRAIL_STEP = 0.02       # m  이만큼 움직였을 때만 자취 점 추가


def _rviz_config_path():
    """설치된 share → 소스 트리 순으로 traj.rviz 를 찾는다."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('crazyflie_traj'),
                            'config', RVIZ_CONFIG)
        if os.path.exists(path):
            return path
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'config', RVIZ_CONFIG)


def open_rviz():
    """traj.rviz 로 rviz2 를 띄운다. 이미 떠 있으면 그대로 둔다."""
    running = subprocess.run(['pgrep', '-f', f'rviz2.*{RVIZ_CONFIG}'],
                             stdout=subprocess.DEVNULL).returncode == 0
    if running:
        return
    # 비행 스크립트가 끝나도 rviz 는 남도록 세션을 분리한다
    subprocess.Popen(['rviz2', '-d', _rviz_config_path()],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                     start_new_session=True)
    print('  · rviz2 실행 (traj.rviz)')


class FlightViz:
    """계획 궤적·실궤적 퍼블리셔. node 는 spin 되는 rclpy 노드(swarm.allcfs)."""

    def __init__(self, node, shape_fn, offset, robot_frame, frame_id='world',
                 samples=800):
        from nav_msgs.msg import Path
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from tf2_msgs.msg import TFMessage

        self.node = node
        self.frame_id = frame_id
        self.robot_frame = robot_frame
        self.trail = []

        pos, _, _ = shape_fn(np.linspace(0.0, 1.0, samples))
        self.plan = pos + offset

        # transient_local: rviz 를 나중에 띄워도 계획선을 받는다
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_path = node.create_publisher(Path, 'traj/path', qos)
        self.pub_flown = node.create_publisher(Path, 'traj/flown', qos)
        node.create_subscription(TFMessage, '/tf', self._on_tf, 20)
        node.create_timer(0.2, self.publish_flown)

        self.publish_flown()                   # 이전 비행 자취를 비운다
        self.pub_path.publish(self._path(self.plan))

    def _path(self, points):
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path
        stamp = self.node.get_clock().now().to_msg()
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = stamp
        for p in points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(p[0])
            ps.pose.position.y = float(p[1])
            ps.pose.position.z = float(p[2])
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg

    def _on_tf(self, msg):
        for t in msg.transforms:
            if t.child_frame_id != self.robot_frame:
                continue
            tr = t.transform.translation
            q = (tr.x, tr.y, tr.z)
            if not self.trail or math.dist(q, self.trail[-1]) >= TRAIL_STEP:
                self.trail.append(q)

    def publish_flown(self):
        self.pub_flown.publish(self._path(self.trail))
