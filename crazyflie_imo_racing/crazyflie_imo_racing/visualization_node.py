"""Read-only RViz displays; ground truth never enters the flight estimator."""
from collections import deque
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster



class Visualization(Node):
    def __init__(self):
        super().__init__('imo_visualization')
        defaults = dict(robot='cf231', backend='sim', initial_position=[0., 0., 0.])
        c = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publishers_by_kind = {k: self.create_publisher(Path, '/imo/' + k + '_path', qos)
                                   for k in ('reference', 'estimated', 'ground_truth')}
        self.markers = self.create_publisher(MarkerArray, '/imo/markers', qos)
        self.tf = TransformBroadcaster(self)
        self.paths = {k: deque(maxlen=6000) for k in ('estimated', 'ground_truth')}
        self.pose = PoseStamped()
        self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z = c['initial_position']
        self.pose.pose.orientation.w = 1.
        self.have_estimate = False
        self.have_truth = False
        self.simulation = c['backend'] == 'sim'
        self.status = 'waiting'
        self.robot = c['robot']
        self.reference = Path()
        self.reference.header.frame_id = 'world'
        self.create_subscription(Path, '/imo/planned_path', self.set_reference, qos)
        self.create_subscription(Odometry, '/imo/odometry', self.estimate, 10)
        self.create_subscription(Odometry, '/imo/ground_truth', self.truth, 10)
        self.create_subscription(PoseStamped, '/' + self.robot + '/pose', self.bootstrap, qos_profile_sensor_data)
        self.create_subscription(String, '/imo/status', self.set_status, qos)
        self.create_timer(.1, self.publish)

    def set_reference(self, msg):
        # A new flight owns its plan; retain it after the flight process exits.
        self.reference = msg
        for points in self.paths.values():
            points.clear()
        self.have_estimate = False
        self.status = 'calibrating'

    def set_status(self, msg):
        self.status = msg.data

    def bootstrap(self, msg):
        if (self.simulation and not self.have_truth) or (not self.simulation and not self.have_estimate):
            self.pose = msg

    def append(self, kind, msg):
        pose = PoseStamped(header=msg.header, pose=msg.pose.pose)
        points = self.paths[kind]
        p = pose.pose.position
        if not points or sum((getattr(p, a) - getattr(points[-1].pose.position, a))**2
                             for a in ('x', 'y', 'z')) >= .005**2:
            points.append(pose)
        return pose

    def estimate(self, msg):
        pose = self.append('estimated', msg)
        if not self.simulation:
            self.pose = pose
        self.have_estimate = True

    def truth(self, msg):
        if self.simulation:
            self.pose = self.append('ground_truth', msg)
            self.have_truth = True

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        self.reference.header.stamp = stamp
        self.publishers_by_kind['reference'].publish(self.reference)
        for kind, points in self.paths.items():
            path = Path()
            path.header.frame_id, path.header.stamp = 'world', stamp
            path.poses = list(points)
            self.publishers_by_kind[kind].publish(path)
        transform = TransformStamped()
        transform.header.frame_id, transform.header.stamp = 'world', stamp
        transform.child_frame_id = self.robot + '/imo_view'
        transform.transform.translation.x = self.pose.pose.position.x
        transform.transform.translation.y = self.pose.pose.position.y
        transform.transform.translation.z = self.pose.pose.position.z
        transform.transform.rotation = self.pose.pose.orientation
        self.tf.sendTransform(transform)
        body = Marker()
        body.header.frame_id, body.header.stamp = transform.child_frame_id, stamp
        body.ns, body.id, body.type, body.action = 'drone', 0, Marker.LINE_LIST, Marker.ADD
        body.pose.orientation.w = 1.
        body.scale.x = .018
        body.color.r, body.color.g, body.color.b, body.color.a = (
            (1., .57, .12, 1.) if self.simulation else (.3, 1., .3, 1.))
        body.points = [Point(x=x, y=y, z=0.) for x, y in
                       [(-.12, -.12), (.12, .12), (-.12, .12), (.12, -.12)]]
        nose = deepcopy(body)
        nose.id, nose.type = 1, Marker.ARROW
        nose.points = [Point(), Point(x=.25)]
        nose.scale.x, nose.scale.y, nose.scale.z = .025, .05, .06
        label = Marker()
        label.header.frame_id, label.header.stamp = 'world', stamp
        label.ns, label.id, label.type, label.action = 'status', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose = deepcopy(self.pose.pose)
        label.pose.position.z += .35
        label.scale.z = .15
        label.color.r = label.color.g = label.color.b = label.color.a = 1.
        label.text = self.robot + ': ' + self.status
        self.markers.publish(MarkerArray(markers=[body, nose, label]))


def main(args=None):
    rclpy.init(args=args)
    node = Visualization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
