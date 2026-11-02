"""Straight path tracking and state transitions, independent of ROS."""
from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class Config:
    goal_x: float
    goal_y: float
    control_rate_hz: float = 20.0
    path_speed_m_s: float = 0.3
    min_linear_speed_m_s: float = 0.05
    max_linear_speed_m_s: float = 0.5
    cross_track_gain: float = 1.0
    heading_gain: float = 1.0
    max_angular_speed_rad_s: float = 0.3
    init_timeout_sec: float = 10.0
    position_timeout_sec: float = 0.6
    heading_timeout_sec: float = 0.6
    init_sample_count: int = 10
    goal_tolerance_m: float = 0.1
    goal_hold_sec: float = 2.0

    def __post_init__(self):
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, bool) or not math.isfinite(value):
                raise ValueError(f'{f.name} must be finite')
            if f.name not in ('goal_x', 'goal_y') and value <= 0:
                raise ValueError(f'{f.name} must be positive')
        if not isinstance(self.init_sample_count, int):
            raise ValueError('init_sample_count must be an integer')
        if self.min_linear_speed_m_s > self.max_linear_speed_m_s:
            raise ValueError('minimum speed exceeds maximum')


def track(a, goal, pose, c):
    dx, dy = goal[0] - a[0], goal[1] - a[1]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return (0., 0., 0.)
    tx, ty = dx / length, dy / length
    x, y, yaw = pose
    cross = (x - a[0]) * -ty + (y - a[1]) * tx
    remaining = (goal[0] - x) * tx + (goal[1] - y) * ty
    speed = c.path_speed_m_s if remaining >= 0 else -c.path_speed_m_s
    vx = speed * tx + c.cross_track_gain * cross * ty
    vy = speed * ty - c.cross_track_gain * cross * tx
    size = math.hypot(vx, vy)
    if size:
        scale = min(c.max_linear_speed_m_s, max(c.min_linear_speed_m_s, size)) / size
        vx, vy = vx * scale, vy * scale
    error = math.atan2(math.sin(math.atan2(dy, dx) - yaw),
                       math.cos(math.atan2(dy, dx) - yaw))
    omega = max(-c.max_angular_speed_rad_s,
                min(c.max_angular_speed_rad_s, c.heading_gain * error))
    return (math.cos(yaw) * vx + math.sin(yaw) * vy,
            math.sin(yaw) * vx - math.cos(yaw) * vy, omega)


class Controller:
    def __init__(self, config, now):
        self.c = config
        self.state = 'INIT'
        self.reason = ''
        self.init_started = now
        self.samples = []
        self.a = None
        self.pose = None
        self.stamp = None
        self.received = None
        self.hold_started = None
        self.last_now = now

    @property
    def timeout(self):
        return min(self.c.position_timeout_sec, self.c.heading_timeout_sec)

    def fail(self, reason):
        if self.state == 'COMPLETE':
            return
        self.state = 'ERROR'
        self.reason = reason
        self.samples.clear()
        self.a = None
        self.hold_started = None

    def _clock(self, now):
        if now < self.last_now:
            self.fail('clock_backwards')
            self.stamp = None
            self.received = None
        self.last_now = now

    def tick(self, now):
        self._clock(now)
        if self.state == 'COMPLETE':
            return (0., 0., 0.)
        if self.received is not None and (
                now - self.received >= self.timeout or now - self.stamp >= self.timeout):
            self.fail('pose_timeout')
        if self.state == 'INIT' and now - self.init_started >= self.c.init_timeout_sec:
            self.fail('init_timeout')
        if self.state == 'DRIVE' and math.hypot(
                self.c.goal_x - self.a[0], self.c.goal_y - self.a[1]) < 1e-9:
            self.fail('degenerate_path')
        if self.state == 'DRIVE':
            return track(self.a, (self.c.goal_x, self.c.goal_y), self.pose, self.c)
        return (0., 0., 0.)

    def receive(self, x, y, yaw, stamp, now):
        self.tick(now)  # Expiry must be checked even if a new message just arrived.
        if self.state == 'COMPLETE':
            return False
        if not all(math.isfinite(v) for v in (x, y, yaw, stamp)):
            self.fail('nonfinite_pose')
            return False
        if stamp > now or now - stamp >= self.timeout:
            self.fail('pose_timestamp')
            return False
        if self.stamp is not None and stamp <= self.stamp:
            self.fail('duplicate_or_reversed_stamp')
            return False
        if self.state == 'ERROR':
            self.state = 'INIT'
            self.reason = ''
            self.init_started = now
            self.samples.clear()
            self.a = None
        self.pose = (x, y, yaw)
        self.stamp, self.received = stamp, now
        if self.state == 'INIT':
            self.samples.append((x, y))
            if len(self.samples) < self.c.init_sample_count:
                return True
            self.a = tuple(sum(p[i] for p in self.samples) / len(self.samples) for i in (0, 1))
            self.state = 'DRIVE'
        distance = math.hypot(self.c.goal_x - x, self.c.goal_y - y)
        # An initially coincident A/B requires no path division.
        if distance <= self.c.goal_tolerance_m:
            if self.state != 'GOAL_CHECK':
                self.hold_started = stamp
            self.state = 'GOAL_CHECK'
            if stamp - self.hold_started >= self.c.goal_hold_sec:
                self.state = 'COMPLETE'
        else:
            self.hold_started = None
            self.state = 'DRIVE'
        return True
