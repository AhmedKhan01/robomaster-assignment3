"""Raw SDK snapshot buffer; no filtering, calibration or motion commands."""
import math
import threading
import time


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class OdometryBuffer:
    def __init__(self, y_sign=-1.0, yaw_sign=-1.0):
        if y_sign not in (-1.0, 1.0) or yaw_sign not in (-1.0, 1.0):
            raise ValueError('Odometry signs must be -1 or +1')
        self.y_sign, self.yaw_sign = y_sign, yaw_sign
        self.lock = threading.Lock()
        self.samples = {}
        self.counts = {}
        self.yaw_rate = None

    def put(self, channel, data, ros_s, monotonic_s=None):
        n = {'position': 3, 'attitude': 3, 'velocity': 6}[channel]
        if len(data) != n:
            return False
        values = tuple(float(v) for v in data)
        now = time.monotonic() if monotonic_s is None else monotonic_s
        if not all(math.isfinite(v) for v in (*values, ros_s, now)):
            return False
        with self.lock:
            if channel == 'attitude':
                previous = self.samples.get(channel)
                self.yaw_rate = None
                if previous:
                    dt = now-previous['monotonic_s']
                    if 0.005 <= dt <= 0.4:
                        angle = self.yaw_sign*math.radians(values[0]-previous['values'][0])
                        self.yaw_rate = wrap(angle)/dt
            seq = self.counts.get(channel, 0)+1
            self.counts[channel] = seq
            self.samples[channel] = dict(values=values, monotonic_s=now, receipt_ros_s=ros_s, seq=seq)
        return True

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            samples = {k: dict(v, age_s=now-v['monotonic_s']) for k,v in self.samples.items()}
            yaw_rate = self.yaw_rate
        return samples, yaw_rate

    def pose(self, now=None, max_age=0.4, max_skew=0.15):
        samples, yaw_rate = self.snapshot(now)
        p, a = samples.get('position'), samples.get('attitude')
        if not p or not a:
            return None
        if any(not 0 <= s['age_s'] <= max_age for s in (p,a)):
            return None
        if abs(p['monotonic_s']-a['monotonic_s']) > max_skew:
            return None
        x, sdk_y, unused_third_position_field = p['values']
        yaw = self.yaw_sign*math.radians(a['values'][0])
        v = samples.get('velocity')
        velocity_valid = bool(v and 0 <= v['age_s'] <= max_age and
                              abs(p['monotonic_s']-v['monotonic_s']) <= max_skew)
        return dict(x=x, y=self.y_sign*sdk_y, yaw=yaw, position_seq=p['seq'],
            stamp_ros_s=p['receipt_ros_s'], samples=samples,
            vx=v['values'][3] if velocity_valid else math.nan,
            vy=self.y_sign*v['values'][4] if velocity_valid else math.nan,
            yaw_rate=yaw_rate if yaw_rate is not None else math.nan,
            velocity_valid=velocity_valid)
