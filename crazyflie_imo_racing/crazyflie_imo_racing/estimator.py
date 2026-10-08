"""Streaming adapter around the original IMO FilterRunner, without GT feedback."""
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from .motor import thrust_acceleration


class OnlineEstimator:
    def __init__(self, repo, checkpoint, parameters, initial_position=(0, 0, 0),
                 initial_yaw=0., mass=.034, thrust_scale=1., accel_scale=1.,
                 accel_bias=(0, 0, 0), gyro_bias=(0, 0, 0),
                 update_freq=20., max_gap=.05, tuning=None, innovation_gate=7.815,
                 max_position_correction=.1, max_velocity_correction=.1,
                 max_attitude_correction_deg=.5):
        src = Path(repo).expanduser().resolve() / 'src'
        if not (src / 'filter/python/src/filter_runner.py').is_file():
            raise ValueError(f'Invalid IMO repository: {repo}')
        sys.path.insert(0, str(src))
        import torch
        from filter.python.src.filter_runner import FilterRunner
        from filter.python.src.utils.dotdict import dotdict
        torch.set_num_threads(1)
        self.torch = torch
        cfg = dict(g_norm=9.81, sigma_na=.1, sigma_ng=.001, sigma_nba=.01,
                   sigma_nbg=.0001, init_attitude_sigma=np.deg2rad(.1),
                   init_yaw_sigma=np.deg2rad(1), init_vel_sigma=.01,
                   init_pos_sigma=.01, init_bg_sigma=.0001, init_ba_sigma=.0001,
                   zero_vel_sigma=.01,
                   use_const_cov=True, const_cov_val_x=.0231,
                   const_cov_val_y=.0244, const_cov_val_z=.0182,
                   meascov_scale=1., mahalanobis_factor=-1., mahalanobis_fail_scale=0.)
        cfg.update(tuning or {})
        self.runner = FilterRunner(str(checkpoint), str(parameters), float(update_freq),
                                   dotdict(cfg), {'accel_bias': np.array(accel_bias),
                                                 'gyro_bias': np.array(gyro_bias)}, True)
        # Unlike the offline EKF's delayed convergence gate, reject statistically
        # inconsistent learned measurements from the very first update. This is
        # essential for takeoff/hover outside the cruise-only training data.
        self.rejected_updates = 0
        original_update = self.runner.filter.apply_update

        def guarded_update(innovation, jacobian, noise):
            S = jacobian @ self.runner.filter.Sigma @ jacobian.T + noise
            weighted = np.linalg.solve(S, innovation)
            nis = float((innovation.T @ weighted).item())
            if not np.isfinite(nis) or (innovation_gate > 0 and nis > innovation_gate):
                self.rejected_updates += 1
                return False
            # A relative displacement can otherwise cause a very large global
            # state jump after prolonged rejection/covariance growth. Never
            # silently turn that jump into a control command.
            delta = (self.runner.filter.Sigma @ jacobian.T @ weighted)[-15:].ravel()
            limits = ((delta[:3], np.deg2rad(max_attitude_correction_deg)),
                      (delta[3:6], max_velocity_correction), (delta[6:9], max_position_correction))
            if any(limit > 0 and np.linalg.norm(x) > limit for x, limit in limits):
                self.rejected_updates += 1
                return False
            return original_update(innovation, jacobian, noise)

        self.runner.filter.apply_update = guarded_update
        self.initial_position = np.array(initial_position, dtype=float)
        self.initial_rotation = Rotation.from_euler('z', initial_yaw).as_matrix()
        self.mass, self.thrust_scale, self.accel_scale = mass, thrust_scale, accel_scale
        self.max_gap_us = int(max_gap * 1e6)
        self.previous = None
        self.next_us = None
        self.updates = 0
        self.samples = 0

    def set_initial_attitude(self, rotation):
        if self.previous is not None:
            raise RuntimeError('Cannot change initial attitude after initialization')
        self.initial_rotation = np.asarray(rotation).reshape(3, 3)

    def feed_raw(self, t_us, imu, pwm):
        """acc.xyz in g, gyro.xyz in deg/s, four actual logged motor PWMs."""
        imu, pwm = np.asarray(imu, float), np.asarray(pwm, float)
        if imu.shape != (6,) or pwm.shape != (4,):
            raise ValueError('Expected six IMU fields and four PWM fields')
        if not np.isfinite(imu).all() or not np.isfinite(pwm).all():
            raise ValueError('Non-finite sensor sample')
        if np.any(pwm < 0) or np.any(pwm > 65535):
            raise ValueError('PWM outside firmware range')
        return self.feed_si(t_us, np.deg2rad(imu[3:]), imu[:3]*9.81*self.accel_scale,
                            thrust_acceleration(pwm, self.mass, self.thrust_scale))

    def feed_si(self, t_us, gyro, accel, thrust):
        """Resample all channels onto an integer-microsecond network clock.

        The offline runner advances only one interpolation tick per call. Feeding
        raw jitter or dropped samples directly would corrupt its clone timestamps.
        Interpolation uses only the two received endpoints, never future samples.
        """
        t_us = int(t_us)
        values = np.concatenate([gyro, accel, thrust]).astype(float)
        if values.shape != (9,) or not np.isfinite(values).all():
            raise ValueError('Invalid SI sample')
        r = self.runner
        if self.previous is None:
            r.filter.initialize_with_state(t_us, self.initial_rotation,
                np.zeros((3, 1)), self.initial_position.reshape(3, 1),
                r.icalib.accelBias, r.icalib.gyroBias)
            # Seed thrust (not accelerometer) explicitly; avoid the offline
            # runner's no-GT initialization branch, which seeds the wrong channel.
            r.next_interp_t_us = t_us
            r._add_interpolated_inputs_to_buffer(values[6:].reshape(3, 1),
                values[:3].reshape(3, 1)-r.icalib.gyroBias, t_us)
            r.last_t_us = t_us
            r.last_acc = values[3:6].reshape(3, 1)-r.icalib.accelBias
            r.last_gyr = values[:3].reshape(3, 1)-r.icalib.gyroBias
            r.last_thrust = values[6:].reshape(3, 1)
            r.next_aug_t_us = t_us + r.dt_update_us
            self.next_us = t_us + r.dt_interp_us
        else:
            prev_t, prev = self.previous
            if t_us <= prev_t:
                raise ValueError('Non-monotonic sensor timestamp')
            if t_us - prev_t > self.max_gap_us:
                raise ValueError('Sensor gap exceeds max_gap; estimator reset required')
            with self.torch.inference_mode():
                while self.next_us <= t_us:
                    x = prev + (values-prev) * ((self.next_us-prev_t)/(t_us-prev_t))
                    updated = r.on_imu_measurement(int(self.next_us),
                        x[:3].reshape(3, 1), x[3:6].reshape(3, 1), x[6:].reshape(3, 1))
                    self.updates += int(updated)
                    self.samples += 1
                    self.next_us += r.dt_interp_us
        self.previous = (t_us, values)
        state = self.state()
        if not all(np.isfinite(v).all() for v in state.values()):
            raise ValueError('Non-finite EKF state')
        return state

    def state(self):
        R, v, p, ba, bg = self.runner.filter.get_evolving_state()
        return dict(rotation=R.copy(), position=p.ravel().copy(), velocity=v.ravel().copy())
