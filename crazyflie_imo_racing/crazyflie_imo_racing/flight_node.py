"""ROS transport, flight lifecycle, IMU/PWM estimation and host control."""
import csv
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
from crazyflie_interfaces.msg import LogDataGeneric, Status
from crazyflie_interfaces.srv import Arm, Takeoff, GoTo, Land
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml
from .controller import PositionController
from .estimator import OnlineEstimator
from .sensors import SensorPairer
from .trajectory import Trajectory, FlightPlan, transition


class FlightNode(Node):
    def __init__(self):
        super().__init__('imo_flight')
        share = Path(get_package_share_directory('crazyflie_imo_racing'))
        defaults = yaml.safe_load((share/'config/flight.yaml').read_text())['imo_flight']['ros__parameters']
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        c = self.cfg
        for key in ('takeoff_duration', 'settle_duration', 'land_duration', 'mass', 'thrust_scale',
                    'accel_scale', 'sensor_max_gap', 'watchdog_seconds', 'max_distance', 'max_speed',
                    'max_tracking_error', 'update_freq'):
            if not np.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        for key in ('initial_position', 'kp', 'kd', 'displacement_sigma'):
            if len(c[key]) != 3 or not np.isfinite(c[key]).all():
                raise ValueError(f'{key} must have three finite values')
        if any(x <= 0 for x in c['displacement_sigma']):
            raise ValueError('displacement_sigma must be positive')
        if c['calibration_samples'] < 20 or not 0 < c['max_tilt_deg'] <= 40 or not 10000 < c['max_pwm'] <= 60000:
            raise ValueError('Invalid calibration_samples, max_tilt_deg or max_pwm')
        if c['backend'] not in ('sim', 'cflib'):
            raise ValueError('Supported backends: sim, cflib')
        if c['firmware_controller'] not in ('pid', 'mellinger'):
            raise ValueError('firmware_controller must be pid or mellinger')
        if c['backend'] == 'cflib' and c['sensor_profile'] != 'hardware':
            raise ValueError('Use flight_hardware.yaml / sensor_profile=hardware; ideal sim tuning is not for hardware')
        if c['backend'] != 'sim' and c['autostart']:
            raise ValueError('Hardware requires explicit /imo/start service after calibration')
        if bool(c['checkpoint']) != bool(c['model_parameters']):
            raise ValueError('Custom checkpoint and model_parameters must be supplied together')
        self.checkpoint = Path(c['checkpoint']).expanduser() if c['checkpoint'] else share/'models/cfsim_best.pt'
        self.parameters = Path(c['model_parameters']).expanduser() if c['model_parameters'] else share/'models/cfsim_best.json'
        self.origin = np.array(c['initial_position'])
        self.trajectory = Trajectory(c['trajectory'], self.origin, c['height'], c['radius'],
                                    c['period'], c['laps'], c['trajectory_csv'], c['timescale'],
                                    c['yaw_mode'], np.deg2rad(c['initial_yaw_deg']), c['ramp_duration'])
        self.plan = FlightPlan(self.trajectory, c['takeoff_duration'], c['settle_duration'], c['land_duration'])
        self.controller = PositionController(c['mass'], c['thrust_scale'], c['kp'], c['kd'],
                                             c['max_tilt_deg'], c['max_pwm'])
        # Warm network + Numba before any command can arm the aircraft.
        self.get_logger().info(f'Loading {self.checkpoint}; warming inference/EKF before subscribing')
        self.estimator = self.make_estimator()
        for i in range(80):
            self.estimator.feed_si(i*10000, np.zeros(3), np.array([0, 0, 9.81]), np.array([0, 0, 9.81]))
        self.estimator = self.make_estimator()
        prefix = '/'+c['robot']
        self.commands = self.create_publisher(Twist, prefix+'/cmd_vel_legacy', 1)
        self.odom = self.create_publisher(Odometry, '/imo/odometry', 10)
        self.status_pub = self.create_publisher(String, '/imo/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.plan_pub = self.create_publisher(RosPath, '/imo/planned_path',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        reference = RosPath()
        reference.header.frame_id = 'world'
        reference.header.stamp = self.get_clock().now().to_msg()
        for i in range(1001):
            position, _, _, yaw, _ = self.trajectory.sample(self.trajectory.duration * i / 1000)
            pose = PoseStamped()
            pose.header = reference.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, position)
            pose.pose.orientation.z = float(np.sin(yaw / 2))
            pose.pose.orientation.w = float(np.cos(yaw / 2))
            reference.poses.append(pose)
        self.plan_pub.publish(reference)
        self.arm_client = self.create_client(Arm, prefix+'/arm')
        self.takeoff_client = self.create_client(Takeoff, prefix+'/takeoff')
        self.goto_client = self.create_client(GoTo, prefix+'/go_to')
        self.land_client = self.create_client(Land, prefix+'/land')
        sensor_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LogDataGeneric, prefix+'/imu_raw', lambda m: self.sensor('imu', m), sensor_qos)
        self.create_subscription(LogDataGeneric, prefix+'/motor_pwm', lambda m: self.sensor('pwm', m), sensor_qos)
        self.create_subscription(Status, prefix+'/status', self.battery_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, prefix+'/pose', self.pose_cb, qos_profile_sensor_data)
        self.create_service(Trigger, '/imo/start', self.start_cb)
        self.create_service(Trigger, '/imo/land', self.land_cb)
        self.create_service(Trigger, '/imo/stop', self.stop_cb)
        self.create_timer(.05, self.watchdog)
        self.pairer = SensorPairer(c['sensor_pair_tolerance_ms'])
        self.calibration = []
        self.ready = self.active = self.done = False
        self.phase = 'calibrating'
        self.start_us = None
        self.last_sensor_wall = None
        self.battery = None
        self.supervisor_info = 0
        self.last_pwm = np.zeros(4)
        self.arm_future = None
        self.takeoff_future = None
        self.align_future = None
        self.align_start_us = None
        self.bootstrap_land_us = None
        self.handoff_us = None
        self.bootstrap_pose = None
        self.bootstrap_velocity = np.zeros(3)
        self.pose_wall = None
        self.early_land = None
        self.last_state = None
        self.last_us = None
        folder = Path(c['log_dir']).expanduser()/(time.strftime('%Y%m%d_%H%M%S')+f'_{time.time_ns()%1000000000:09d}')
        folder.mkdir(parents=True, exist_ok=False)
        self.log_file = (folder/'flight.csv').open('w')
        self.log = csv.writer(self.log_file)
        self.log.writerow(['t_us', 'elapsed', 'phase', 'px', 'py', 'pz', 'vx', 'vy', 'vz',
                           'ref_x', 'ref_y', 'ref_z', 'roll_deg', 'pitch_deg', 'yawrate_deg', 'pwm', 'updates',
                           'qx', 'qy', 'qz', 'qw'])
        self.raw_file = (folder/'sensors.csv').open('w')
        self.raw = csv.writer(self.raw_file)
        self.raw.writerow(['t_us', 'ax_g', 'ay_g', 'az_g', 'gx_dps', 'gy_dps', 'gz_dps', 'm1', 'm2', 'm3', 'm4'])
        self.folder = folder
        import hashlib
        metadata = dict(c, resolved_checkpoint=str(self.checkpoint),
                        checkpoint_sha256=hashlib.sha256(self.checkpoint.read_bytes()).hexdigest())
        (folder/'run.json').write_text(json.dumps(metadata, indent=2))
        self.get_logger().info(f'Logging to {folder}; waiting for stationary IMU/PWM')

    def make_estimator(self, accel_bias=(0, 0, 0), gyro_bias=(0, 0, 0)):
        c = self.cfg
        sigma = c['displacement_sigma']
        tuning = dict(const_cov_val_x=sigma[0], const_cov_val_y=sigma[1], const_cov_val_z=sigma[2],
                      init_attitude_sigma=np.deg2rad(c['initial_attitude_sigma_deg']),
                      init_yaw_sigma=np.deg2rad(c['initial_yaw_sigma_deg']),
                      init_vel_sigma=c['initial_velocity_sigma'])
        tuning.update({k: c[k] for k in ('sigma_na', 'sigma_ng', 'sigma_nba', 'sigma_nbg')})
        return OnlineEstimator(c['imo_repo'], self.checkpoint, self.parameters,
            c['initial_position'], np.deg2rad(c['initial_yaw_deg']), c['mass'], c['thrust_scale'],
            c['accel_scale'], accel_bias, gyro_bias, c['update_freq'], c['sensor_max_gap'],
            tuning, c['innovation_gate'], c['max_position_correction'],
            c['max_velocity_correction'], c['max_attitude_correction_deg'])

    def battery_cb(self, msg):
        self.battery = (float(msg.battery_voltage), time.monotonic())
        self.supervisor_info = msg.supervisor_info
        if self.active and self.supervisor_info & (Status.SUPERVISOR_INFO_IS_TUMBLED | Status.SUPERVISOR_INFO_IS_LOCKED):
            self.fail('Firmware supervisor reports tumbled/locked')
        if self.active and self.battery[0] < self.cfg['min_battery'] and self.early_land is None:
            self.begin_landing()

    def pose_cb(self, msg):
        # Once control transfers to IMO, onboard pose is deliberately ignored.
        if self.handoff_us is not None:
            return
        p = msg.pose.position; q = msg.pose.orientation
        xyz = np.array([p.x, p.y, p.z])
        quat = np.array([q.x, q.y, q.z, q.w])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if not np.isfinite(np.r_[xyz, quat]).all() or np.linalg.norm(quat) < .5:
            return
        if self.bootstrap_pose is not None:
            dt = stamp-self.bootstrap_pose[0]
            if .001 < dt < .2:
                self.bootstrap_velocity = .8*self.bootstrap_velocity + .2*(xyz-self.bootstrap_pose[1])/dt
        self.bootstrap_pose = (stamp, xyz, Rotation.from_quat(quat).as_matrix())
        self.pose_wall = time.monotonic()

    def sensor(self, channel, msg):
        if self.done:
            return
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            age = self.get_clock().now().nanoseconds*1e-9-stamp
            if self.active and age > self.cfg['watchdog_seconds']:
                raise ValueError('Sensor callback backlog exceeds latency limit')
            for t, imu, pwm in self.pairer.add(channel, msg.timestamp, msg.values):
                if imu.shape != (6,) or pwm.shape != (4,) or not np.isfinite(np.r_[imu, pwm]).all():
                    raise ValueError('Malformed sensor log fields')
                self.last_sensor_wall = time.monotonic()
                self.last_pwm = pwm
                self.raw.writerow([t, *imu, *pwm])
                self.last_us = t
                if not self.ready:
                    self.calibrate(imu, pwm)
                    continue
                if self.arm_future is not None and self.arm_future.done():
                    self.arm_future.result()  # propagate service failure
                    self.arm_future = None
                    self.active = True
                    self.start_us = t
                    req = Takeoff.Request()
                    req.height = float(self.trajectory.home[2])
                    duration = self.cfg['takeoff_duration']
                    req.duration.sec = int(duration)
                    req.duration.nanosec = int((duration-int(duration))*1e9)
                    self.takeoff_future = self.takeoff_client.call_async(req)
                    self.set_phase('takeoff')
                if not self.active:
                    if self.cfg['autostart'] and self.ready:
                        self.request_start()
                    continue
                if self.handoff_us is None:
                    self.bootstrap(t)
                    if self.handoff_us is None:
                        continue
                state = self.estimator.feed_raw(t, imu, pwm)
                if self.estimator.samples == 0:
                    self.estimator.runner.filter.state.s_v = self.bootstrap_velocity.reshape(3, 1).copy()
                    state = self.estimator.state()
                self.last_state = state
                self.control(t, state)
        except Exception as exc:
            self.fail(str(exc))

    def bootstrap(self, t):
        if self.bootstrap_land_us is not None:
            if (t-self.bootstrap_land_us)/1e6 >= self.cfg['land_duration']+1.:
                self.set_phase('complete'); self.finish()
            return
        if self.takeoff_future is not None:
            if not self.takeoff_future.done():
                return
            self.takeoff_future.result()
            self.takeoff_future = None
            self.start_us = t  # time the stabilization interval from service ack
        elapsed = (t-self.start_us)/1e6
        if elapsed < self.cfg['takeoff_duration']:
            return
        if self.align_start_us is None:
            req = GoTo.Request()
            req.goal.x, req.goal.y, req.goal.z = map(float, self.trajectory.home)
            req.yaw = float(self.trajectory.sample(0)[3])
            duration = self.cfg['settle_duration']
            req.duration.sec = int(duration)
            req.duration.nanosec = int((duration-int(duration))*1e9)
            self.align_future = self.goto_client.call_async(req)
            self.align_start_us = t
        self.set_phase('settle')
        if self.align_future is not None:
            if not self.align_future.done():
                return
            self.align_future.result()
            self.align_future = None
            self.align_start_us = t
        if (t-self.align_start_us)/1e6 < self.cfg['settle_duration']+1.:
            return
        if self.bootstrap_pose is None or time.monotonic()-self.pose_wall > .2:
            raise ValueError('Fresh onboard pose required for one-time IMO handoff')
        _, p, R = self.bootstrap_pose
        if np.linalg.norm(p-self.trajectory.home) > .15 or np.linalg.norm(self.bootstrap_velocity) > .15:
            raise ValueError('Onboard takeoff has not settled at trajectory start')
        self.estimator.initial_position = p.copy()
        self.estimator.set_initial_attitude(R)
        self.handoff_us = t
        (self.folder/'handoff.json').write_text(json.dumps(dict(t_us=t, position=p.tolist(),
            velocity=self.bootstrap_velocity.tolist(), rotation=R.tolist(),
            source='one-time onboard pose after takeoff/settle; ignored after handoff'), indent=2))
        self.set_phase('trajectory')

    def calibrate(self, imu, pwm):
        # Explicit stationary-on-ground initialization; no pose or GT topic.
        if np.max(pwm) > 1000 or np.linalg.norm(imu[3:]) > 5 or not .85 < np.linalg.norm(imu[:3]) < 1.15:
            self.calibration.clear()
            return
        self.calibration.append(imu.copy())
        if len(self.calibration) < self.cfg['calibration_samples']:
            return
        data = np.array(self.calibration)
        if np.max(data[:, :3].std(axis=0)) > .02:
            self.calibration.clear()
            return
        acc = data[:, :3].mean(axis=0)*9.81*self.cfg['accel_scale']
        bg = np.deg2rad(data[:, 3:].mean(axis=0))
        # Gravity fixes roll/pitch; yaw is the configured launch heading.
        tilt, _ = Rotation.align_vectors([[0, 0, 1]], [acc/np.linalg.norm(acc)])
        R = Rotation.from_euler('z', np.deg2rad(self.cfg['initial_yaw_deg'])).as_matrix() @ tilt.as_matrix()
        ba = acc - R.T@np.array([0, 0, 9.81])
        self.estimator = self.make_estimator(ba, bg)
        self.estimator.set_initial_attitude(R)
        (self.folder/'calibration.json').write_text(json.dumps(dict(accel_bias=ba.tolist(),
            gyro_bias=bg.tolist(), rotation=R.tolist()), indent=2))
        self.ready = True
        self.set_phase('ready')

    def request_start(self):
        if not self.ready or self.active or self.arm_future is not None or self.done:
            return False, 'Not ready, already starting, or session finished'
        if self.last_sensor_wall is None or time.monotonic()-self.last_sensor_wall > .2:
            return False, 'No fresh synchronized IMU/PWM'
        if self.cfg['backend'] != 'sim':
            if self.battery is None or time.monotonic()-self.battery[1] > 3 or self.battery[0] < self.cfg['min_battery']:
                return False, 'Fresh sufficient battery status required'
            if self.supervisor_info & (Status.SUPERVISOR_INFO_IS_TUMBLED | Status.SUPERVISOR_INFO_IS_LOCKED):
                return False, 'Firmware is tumbled or locked'
        if np.max(self.last_pwm) > 1000:
            return False, 'Motors must be stopped before starting a new flight'
        if not all(client.service_is_ready() for client in (self.arm_client, self.takeoff_client, self.goto_client, self.land_client)):
            return False, 'Waiting for Crazyflie arm/takeoff/go_to/land services'
        if self.bootstrap_pose is None or time.monotonic()-self.pose_wall > .2:
            return False, 'Fresh onboard pose required for takeoff and handoff'
        self.commands.publish(Twist())
        req = Arm.Request(); req.arm = True
        self.arm_future = self.arm_client.call_async(req)
        self.set_phase('arming')
        return True, 'Arming; onboard takeoff/settle then IMO trajectory control'

    def start_cb(self, req, response):
        response.success, response.message = self.request_start()
        return response

    def begin_landing(self):
        if self.active and self.handoff_us is None and self.bootstrap_land_us is None:
            req = Land.Request(); req.height = .04
            duration = self.cfg['land_duration']
            req.duration.sec = int(duration)
            req.duration.nanosec = int((duration-int(duration))*1e9)
            self.land_client.call_async(req)
            self.bootstrap_land_us = self.last_us
            self.set_phase('land')
        elif self.active and self.handoff_us is not None and self.last_state is not None:
            self.early_land = (self.last_us, self.last_state['position'].copy())

    def land_cb(self, req, response):
        self.begin_landing()
        response.success = self.early_land is not None or self.bootstrap_land_us is not None
        response.message = 'Landing using IMO position' if response.success else 'Not flying'
        return response

    def stop_cb(self, req, response):
        self.fail('Explicit motor stop requested')
        response.success, response.message = True, 'Motor stop latched; restart session to fly again'
        return response

    def control(self, t, state):
        elapsed = (t-self.handoff_us)/1e6
        phase, ref = self.plan.sample(elapsed+self.plan.takeoff+self.plan.settle)
        if self.early_land is not None:
            start, p = self.early_land
            land_t = (t-start)/1e6
            end = p.copy(); end[2] = self.origin[2]
            phase = 'land' if land_t < self.cfg['land_duration'] else 'complete'
            ref = transition(p, end, land_t, self.cfg['land_duration'])
            yaw = np.arctan2(state['rotation'][1, 0], state['rotation'][0, 0])
            ref = (*ref[:3], float(yaw), 0.)
        self.set_phase(phase)
        if phase == 'complete':
            self.finish()
            return
        c = self.cfg
        if np.linalg.norm(state['position']-self.origin) > c['max_distance']:
            raise ValueError('Estimated position outside configured flight radius')
        if np.linalg.norm(state['velocity']) > c['max_speed']:
            raise ValueError('Estimated speed limit exceeded')
        if np.linalg.norm(state['position']-ref[0]) > c['max_tracking_error']:
            raise ValueError('Tracking error limit exceeded')
        if state['rotation'][2, 2] < np.cos(np.deg2rad(55)):
            raise ValueError('Estimated tilt exceeds 55 degrees')
        command = self.controller.command(state, ref)
        msg = Twist()
        msg.linear.x = command['pitch_deg']; msg.linear.y = command['roll_deg']
        msg.linear.z = command['pwm']; msg.angular.z = command['yaw_rate_deg']
        self.commands.publish(msg)
        odom = Odometry(); odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'world'; odom.child_frame_id = self.cfg['robot']+'/imo'
        p, v = state['position'], state['velocity']
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, p)
        q = Rotation.from_matrix(state['rotation']).as_quat()
        o = odom.pose.pose.orientation
        o.x, o.y, o.z, o.w = map(float, q)
        # Odometry twist is expressed in child_frame_id, not the world frame.
        vb = state['rotation'].T@v
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = map(float, vb)
        covariance = self.estimator.runner.filter.Sigma15
        pose_order = [6, 7, 8, 0, 1, 2]
        odom.pose.covariance = covariance[np.ix_(pose_order, pose_order)].ravel().tolist()
        twist_cov = np.zeros((6, 6))
        twist_cov[:3, :3] = state['rotation'].T@covariance[3:6, 3:6]@state['rotation']
        odom.twist.covariance = twist_cov.ravel().tolist()
        self.odom.publish(odom)
        self.log.writerow([t, elapsed, phase, *p, *v, *ref[0], command['roll_deg'],
                           command['pitch_deg'], command['yaw_rate_deg'], command['pwm'], self.estimator.updates, *q])

    def set_phase(self, phase):
        if self.phase != phase:
            self.phase = phase
            self.get_logger().info(f'Flight: {phase}')
            msg = String(); msg.data = phase; self.status_pub.publish(msg)

    def watchdog(self):
        if self.active or self.arm_future is not None:
            if self.last_sensor_wall is None or time.monotonic()-self.last_sensor_wall > self.cfg['watchdog_seconds']:
                self.fail('Synchronized sensor watchdog expired')
        if self.done:
            self.commands.publish(Twist())  # keep zero command latched until shutdown

    def finish(self):
        self.arm_future = None
        self.commands.publish(Twist())
        if self.arm_client.service_is_ready():
            req = Arm.Request(); req.arm = False; self.arm_client.call_async(req)
        self.active = False
        self.done = True
        self.log_file.flush(); self.raw_file.flush()
        (self.folder/'result.json').write_text(json.dumps(dict(phase=self.phase,
            learned_updates=self.estimator.updates, rejected_updates=self.estimator.rejected_updates,
            dropped_pairs=self.pairer.dropped), indent=2))

    def fail(self, reason):
        if not self.done:
            self.get_logger().error(reason)
            self.set_phase('fault: '+reason)
            self.finish()

    def destroy_node(self):
        if hasattr(self, 'commands') and self.context.ok():
            self.commands.publish(Twist())
        if hasattr(self, 'log_file'):
            self.log_file.close(); self.raw_file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    failed = False
    try:
        node = FlightNode()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
            if node.done and node.cfg['exit_on_complete']:
                # Allow zero command/disarm to leave DDS before exiting.
                for _ in range(5):
                    rclpy.spin_once(node, timeout_sec=.05)
                break
    except KeyboardInterrupt:
        if node:
            node.fail('Process interrupted')
    finally:
        if node:
            failed = node.phase.startswith('fault:')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failed:
        raise SystemExit(1)
