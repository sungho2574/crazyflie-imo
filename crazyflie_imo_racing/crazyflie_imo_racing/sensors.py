"""Bounded pairing by firmware timestamp, not ROS arrival time."""
from collections import deque
import numpy as np


class SensorPairer:
    def __init__(self, tolerance_ms=5, max_pending=200):
        self.imu, self.pwm = deque(), deque()
        self.tolerance = int(tolerance_ms)
        self.max_pending = max_pending
        self.last = {'imu': None, 'pwm': None}
        self.dropped = 0

    def add(self, channel, timestamp_ms, values):
        stamp = int(timestamp_ms)
        if self.last[channel] is not None and stamp <= self.last[channel]:
            raise ValueError(f'{channel} timestamp reversed/reset; restart session')
        self.last[channel] = stamp
        queue = getattr(self, channel)
        queue.append((stamp, np.asarray(values, float)))
        if len(queue) > self.max_pending:
            raise ValueError(f'{channel} synchronization queue overflow')
        pairs = []
        while self.imu and self.pwm:
            ti, imu = self.imu[0]
            tp, pwm = self.pwm[0]
            if abs(ti-tp) <= self.tolerance:
                self.imu.popleft(); self.pwm.popleft()
                pairs.append((ti*1000, imu, pwm))
            elif ti < tp:
                self.imu.popleft(); self.dropped += 1
            else:
                self.pwm.popleft(); self.dropped += 1
        return pairs
