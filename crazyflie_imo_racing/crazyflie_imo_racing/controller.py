"""World position PD + feedforward; firmware retains attitude/rate control."""
import numpy as np
from .motor import collective_pwm


def firmware_attitude(quat_wxyz):
    """Body->world quaternion to firmware roll, legacy pitch, yaw in degrees."""
    w, x, y, z = quat_wxyz
    return np.rad2deg([
        np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
        -np.arcsin(np.clip(2*(w*y-z*x), -1, 1)),
        np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)),
    ])


class PositionController:
    def __init__(self, mass=.034, thrust_scale=1., kp=(2., 2., 4.),
                 kd=(2., 2., 2.5), max_tilt_deg=20., max_pwm=60000):
        self.mass, self.scale = mass, thrust_scale
        self.kp, self.kd = np.array(kp), np.array(kd)
        self.max_tilt = np.deg2rad(max_tilt_deg)
        self.max_pwm = max_pwm

    def command(self, state, reference):
        p, v, a, yaw_ref = reference[:4]
        yaw_rate_ref = reference[4] if len(reference) > 4 else 0.
        R = state['rotation']
        yaw = np.arctan2(R[1, 0], R[0, 0])
        desired = (a + self.kp*(p-state['position'])
                   + self.kd*(v-state['velocity']) + [0, 0, 9.81])
        desired[2] = np.clip(desired[2], 3., 16.)
        lateral = np.linalg.norm(desired[:2])
        limit = desired[2]*np.tan(self.max_tilt)
        if lateral > limit:
            desired[:2] *= limit/lateral
        # Standard body->world Rz(yaw) Ry(pitch) Rx(roll).
        x = np.cos(yaw)*desired[0]+np.sin(yaw)*desired[1]
        y = -np.sin(yaw)*desired[0]+np.cos(yaw)*desired[1]
        roll = np.arctan2(-y, np.hypot(x, desired[2]))
        pitch = np.arctan2(x, desired[2])
        yaw_error = np.arctan2(np.sin(yaw_ref-yaw), np.cos(yaw_ref-yaw))
        # CF legacy firmware pitch has opposite sign from standard RPY. cflib
        # server maps Twist.linear.x -> -pitch; therefore publish +standard pitch.
        pwm = collective_pwm(self.mass*np.linalg.norm(desired), self.scale, self.max_pwm)
        # Firmware legacy yaw rate also has inverted sign (crtp_commander_rpyt.c).
        return dict(roll_deg=float(np.rad2deg(roll)), pitch_deg=float(np.rad2deg(pitch)),
                    yaw_rate_deg=float(-np.clip(np.rad2deg(yaw_rate_ref+2*yaw_error), -180, 180)), pwm=pwm)
