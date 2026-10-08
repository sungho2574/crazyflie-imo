"""Crazyflie firmware PID + Crazyswarm2 numpy dynamics, RPYT transport.

GT is used for simulated attitude sensing/physics and published for evaluation.
No GT position enters the host or firmware position control (all XYZ modes off).
"""
import csv
from pathlib import Path
import time
import numpy as np
import cffirmware as firm
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from crazyflie_sim.crazyflie_sil import CrazyflieSIL
from crazyflie_sim.backend.np import Quadrotor
from crazyflie_sim.sim_data_types import State, Action
from crazyflie_interfaces.msg import LogDataGeneric
from crazyflie_interfaces.srv import Arm, Takeoff, GoTo, Land
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from .motor import rpm_to_force
from .controller import firmware_attitude


class Simulation(Node):
    def __init__(self):
        super().__init__('imo_sim')
        self.robot = self.declare_parameter('robot', 'cf231').value
        pos = self.declare_parameter('initial_position', [0., 0., 0.]).value
        self.t = 0.
        self.ticks = 0
        self.uav = Quadrotor(State(pos))
        controller = self.declare_parameter('firmware_controller', 'mellinger').value
        self.cf = CrazyflieSIL(self.robot, pos, controller, lambda: self.t)
        self.armed = False
        self.cmd = Twist()
        self.last_command = None
        self.command_timeout = self.declare_parameter('command_timeout', .3).value
        prefix = '/'+self.robot
        self.imu_pub = self.create_publisher(LogDataGeneric, prefix+'/imu_raw', qos_profile_sensor_data)
        self.pwm_pub = self.create_publisher(LogDataGeneric, prefix+'/motor_pwm', qos_profile_sensor_data)
        self.gt_pub = self.create_publisher(Odometry, '/imo/ground_truth', 10)
        self.pose_pub = self.create_publisher(PoseStamped, prefix+'/pose', qos_profile_sensor_data)
        self.create_subscription(Twist, prefix+'/cmd_vel_legacy', self.command, 1)
        self.create_service(Arm, prefix+'/arm', self.arm)
        self.create_service(Takeoff, prefix+'/takeoff', self.takeoff)
        self.create_service(GoTo, prefix+'/go_to', self.go_to)
        self.create_service(Land, prefix+'/land', self.land)
        self.bootstrap = False
        self.timer = self.create_timer(.01, self.step)
        logfile = self.declare_parameter('log_file', '').value
        self.log_file = None
        if logfile:
            path = Path(logfile).expanduser(); path.parent.mkdir(parents=True, exist_ok=True)
            self.log_file = path.open('w')
            self.log = csv.writer(self.log_file)
            self.log.writerow(['t_us', 'px', 'py', 'pz', 'vx', 'vy', 'vz', 'qw', 'qx', 'qy', 'qz'])
        self.get_logger().info(f'Crazyflie np physics + firmware {controller}, host RPYT at 100 Hz')

    def arm(self, request, response):
        self.armed = request.arm
        if not self.armed:
            self.cmd = Twist()
        return response

    def command(self, msg):
        self.bootstrap = False
        self.cmd = msg
        self.last_command = time.monotonic()

    def takeoff(self, request, response):
        if self.armed:
            duration = request.duration.sec + request.duration.nanosec*1e-9
            self.cf.takeoff(request.height, duration)
            self.bootstrap = True
        return response

    def go_to(self, request, response):
        if self.armed and self.bootstrap:
            self.cf.goTo([request.goal.x, request.goal.y, request.goal.z], request.yaw,
                         request.duration.sec+request.duration.nanosec*1e-9)
        return response

    def land(self, request, response):
        if self.armed and self.bootstrap:
            self.cf.land(request.height, request.duration.sec+request.duration.nanosec*1e-9)
        return response

    def step(self):
        active = self.armed and (self.bootstrap or (self.last_command is not None
                  and time.monotonic()-self.last_command < self.command_timeout))
        pwm = np.zeros(4)
        action = Action(np.zeros(4))
        acc_samples = []
        gyro_samples = []
        for _ in range(20):  # firmware physics at 2 kHz, sensor logs at 100 Hz
            self.ticks += 1
            self.t = self.ticks*.0005
            self.cf.setState(self.uav.state)
            # The upstream SIL uses rowan's intrinsic XYZ angles, whereas the
            # firmware attitude PID expects standard ZYX roll/pitch/yaw. They
            # disagree as soon as yaw and tilt are both nonzero. Correct only
            # this adapter, leaving the shared simulator untouched.
            roll, pitch, yaw = firmware_attitude(self.uav.state.quat)
            self.cf.state.attitude.roll = float(roll)
            self.cf.state.attitude.pitch = float(pitch)
            self.cf.state.attitude.yaw = float(yaw)
            s = self.cf.setpoint
            if self.bootstrap:
                self.cf.getSetpoint()
            else:
                # No simulator position/velocity is available to the attitude
                # controller after handoff, even as an unused state field.
                for axis in ('x', 'y', 'z'):
                    setattr(self.cf.state.position, axis, 0.)
                    setattr(self.cf.state.velocity, axis, 0.)
                s.mode.x = s.mode.y = s.mode.z = firm.modeDisable
                s.mode.roll = s.mode.pitch = firm.modeAbs
                s.mode.yaw = firm.modeVelocity
                s.mode.quat = firm.modeDisable
                s.attitude.roll = float(self.cmd.linear.y)
                s.attitude.pitch = float(-self.cmd.linear.x)
                s.attitudeRate.yaw = float(-self.cmd.angular.z)
                s.thrust = float(np.clip(self.cmd.linear.z, 0, 60000)) if active else 0.
                self.cf.mode = CrazyflieSIL.MODE_LOW_FULLSTATE
            action = self.cf.executeController()
            if not active or (not self.bootstrap and s.thrust == 0):
                action = Action(np.zeros(4))
                pwm[:] = 0
            else:
                motors = self.cf.motors_thrust_pwm.motors
                pwm = np.array([motors.m1, motors.m2, motors.m3, motors.m4], float)
            self.uav.step(action, .0005)
            force_g = rpm_to_force(action.rpm).sum()/self.uav.mass/9.81
            if self.uav.state.pos[2] <= 0 and force_g < 1:
                force_g = 1.
            acc_samples.append(force_g)
            gyro_samples.append(self.uav.state.omega.copy())
        state = self.uav.state
        # Integrate sensor samples over the reporting interval. A single sample
        # phase-locked to 100 Hz aliases firmware motor oscillations into a DC
        # acceleration bias; actual low-rate IMU logging must be anti-aliased too.
        imu = [0., 0., float(np.mean(acc_samples)), *np.rad2deg(np.mean(gyro_samples, axis=0)).tolist()]
        stamp = self.get_clock().now().to_msg()
        for pub, values in ((self.imu_pub, imu), (self.pwm_pub, pwm.tolist())):
            msg = LogDataGeneric(); msg.header.stamp = stamp
            msg.timestamp = self.ticks//2
            msg.values = values
            pub.publish(msg)
        gt = Odometry(); gt.header.stamp = stamp; gt.header.frame_id = 'world'
        gt.child_frame_id = self.robot+'/truth'
        gt.pose.pose.position.x, gt.pose.pose.position.y, gt.pose.pose.position.z = map(float, state.pos)
        q = gt.pose.pose.orientation
        q.w, q.x, q.y, q.z = map(float, state.quat)
        self.gt_pub.publish(gt)
        pose = PoseStamped(); pose.header = gt.header; pose.pose = gt.pose.pose
        self.pose_pub.publish(pose)
        if self.log_file:
            self.log.writerow([self.ticks*500, *state.pos, *state.vel, *state.quat])

    def destroy_node(self):
        if self.log_file:
            self.log_file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Simulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
