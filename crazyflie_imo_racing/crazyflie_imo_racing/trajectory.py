"""Analytic and Crazyflie polynomial-CSV references in the world frame."""
import numpy as np
from numpy.polynomial.polynomial import polyval, polyder


class Trajectory:
    def __init__(self, kind='circle', origin=(0, 0, 0), height=1., radius=.5,
                 period=12., laps=1, csv='', timescale=1., yaw_mode='forward', initial_yaw=0., ramp=2.):
        if kind not in ('hover', 'circle', 'figure8', 'csv'):
            raise ValueError(f'Unknown trajectory: {kind}')
        if min(height, radius, period, timescale) <= 0 or laps < 1:
            raise ValueError('Trajectory dimensions/timing must be positive')
        self.kind, self.radius, self.period, self.laps = kind, radius, period, laps
        self.origin = np.array(origin, float)
        self.home = self.origin + [0, 0, height]
        self.timescale = timescale
        if yaw_mode not in ('forward', 'fixed'):
            raise ValueError('yaw_mode must be forward or fixed')
        self.yaw_mode, self.initial_yaw = yaw_mode, initial_yaw
        if kind == 'csv':
            self.rows = np.loadtxt(csv, delimiter=',', skiprows=1, ndmin=2)
            if self.rows.shape[1] != 33 or not np.isfinite(self.rows).all():
                raise ValueError('CSV requires duration + 8 coefficients each for x,y,z,yaw')
            if np.any(self.rows[:, 0] <= 0):
                raise ValueError('CSV segment durations must be positive')
            self.ends = np.cumsum(self.rows[:, 0])
            self.period = self.ends[-1] * timescale
            self.home = self._csv(0)[0]
            if np.linalg.norm(self.home[:2]-self.origin[:2]) > .05:
                raise ValueError('CSV start XY must match initial_position')
            # Repeat only truly closed trajectories; never teleport at a lap seam.
            if laps > 1:
                a, b = self._csv(0), self._csv(self.period)
                if any(np.linalg.norm(a[i]-b[i]) > .02 for i in range(3)):
                    raise ValueError('CSV laps require matching endpoint position/velocity/acceleration')
        self.duration = self.period * laps
        self.ramp = min(max(0., ramp), self.duration)
        if kind in ('circle', 'figure8'):
            self.duration += self.ramp

    def _csv(self, t):
        t = np.clip(t / self.timescale, 0, self.ends[-1])
        idx = min(np.searchsorted(self.ends, t, side='right'), len(self.rows)-1)
        local = t - (self.ends[idx-1] if idx else 0)
        c = self.rows[idx, 1:].reshape(4, 8)
        p = np.array([polyval(local, x) for x in c[:3]])
        v = np.array([polyval(local, polyder(x)) for x in c[:3]]) / self.timescale
        a = np.array([polyval(local, polyder(x, 2)) for x in c[:3]]) / self.timescale**2
        return p, v, a, float(polyval(local, c[3])), float(polyval(local, polyder(c[3]))/self.timescale)

    def sample(self, t):
        t = float(np.clip(t, 0, self.duration))
        local = t % self.period if t < self.duration else self.period
        if self.kind == 'csv':
            return self._csv(local)
        if self.kind == 'hover':
            return self.home.copy(), np.zeros(3), np.zeros(3), self.initial_yaw, 0.
        # Match the CFSim collection schedule: two smooth speed ramps with a
        # constant-speed cruise, rather than continuously varying speed.
        w, ramp = 2*np.pi/self.period, self.ramp
        if ramp and t < ramp:
            u = t/ramp
            phase = w*ramp*(u**3-.5*u**4)
            rate = w*u*u*(3-2*u)
            acc = w*6*u*(1-u)/ramp
        elif ramp and t > self.duration-ramp:
            u = (self.duration-t)/ramp
            phase = 2*np.pi*self.laps-w*ramp*(u**3-.5*u**4)
            rate = w*u*u*(3-2*u)
            acc = -w*6*u*(1-u)/ramp
        else:
            phase, rate, acc = w*(t-.5*ramp), w, 0.
        r = self.radius
        if self.kind == 'circle':
            offset = r*np.array([np.cos(phase)-1, np.sin(phase), 0])
            d = r*np.array([-np.sin(phase), np.cos(phase), 0])
            dd = r*np.array([-np.cos(phase), -np.sin(phase), 0])
        else:
            offset = r*np.array([1.2*np.sin(phase), .8*np.sin(2*phase), 0])
            d = r*np.array([1.2*np.cos(phase), 1.6*np.cos(2*phase), 0])
            dd = r*np.array([-1.2*np.sin(phase), -3.2*np.sin(2*phase), 0])
        angle = self.initial_yaw
        C = np.array([[np.cos(angle), -np.sin(angle), 0],
                      [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        offset, d, dd = C@offset, C@d, C@dd
        yaw, yaw_rate = self.initial_yaw, 0.
        if self.yaw_mode == 'forward':
            yaw = float(np.arctan2(d[1], d[0]))
            yaw_rate = float((d[0]*dd[1]-d[1]*dd[0])*rate / np.dot(d[:2], d[:2]))
        return self.home+offset, d*rate, dd*rate**2+d*acc, yaw, yaw_rate


def transition(start, end, t, duration):
    s = np.clip(t / duration, 0, 1)
    d = np.asarray(end)-np.asarray(start)
    return (np.asarray(start)+d*(10*s**3-15*s**4+6*s**5),
            d*(30*s**2-60*s**3+30*s**4)/duration,
            d*(60*s-180*s**2+120*s**3)/duration**2, 0., 0.)


class FlightPlan:
    def __init__(self, trajectory, takeoff=4., settle=2., landing=4.):
        self.traj = trajectory
        self.takeoff, self.settle, self.landing = takeoff, settle, landing
        self.duration = takeoff+settle+trajectory.duration+landing

    def sample(self, t):
        if t < self.takeoff:
            return 'takeoff', transition(self.traj.origin, self.traj.home, t, self.takeoff)
        t -= self.takeoff
        if t < self.settle:
            return 'settle', (self.traj.home.copy(), np.zeros(3), np.zeros(3), self.traj.initial_yaw, 0.)
        t -= self.settle
        if t < self.traj.duration:
            return 'trajectory', self.traj.sample(t)
        t -= self.traj.duration
        final = self.traj.sample(self.traj.duration)
        end = final[0]
        ground = end.copy()
        ground[2] = self.traj.origin[2]
        ref = transition(end, ground, t, self.landing)
        return ('land' if t < self.landing else 'complete', (*ref[:3], final[3], 0.))
