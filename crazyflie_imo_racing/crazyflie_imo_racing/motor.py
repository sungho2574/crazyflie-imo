"""The exact CFSim training PWM -> RPM -> force chain (SI units)."""
import numpy as np


def pwm_to_rpm(pwm):
    pwm = np.asarray(pwm, dtype=float)
    return np.where(pwm < 10000, 0.0, .326535711 * pwm + 3374.95115)


def rpm_to_force(rpm):
    rpm = np.asarray(rpm, dtype=float)
    return np.maximum((2.55077341e-8 * rpm**2 - 4.92422570e-5 * rpm
                       - .151910248) * 9.81 / 1000, 0)


def thrust_acceleration(pwm, mass=.034, scale=1.0):
    return np.array([0., 0., scale * rpm_to_force(pwm_to_rpm(pwm)).sum() / mass])


def collective_pwm(force, scale=1.0, max_pwm=60000):
    """Inverse motor model for equal collective PWM, force is total newtons."""
    grams = max(0., force) / (4 * scale) * 1000 / 9.81
    a, b, c = 2.55077341e-8, -4.92422570e-5, -.151910248 - grams
    rpm = (-b + np.sqrt(b*b - 4*a*c)) / (2*a)
    return float(np.clip((rpm - 3374.95115) / .326535711, 0, max_pwm))
