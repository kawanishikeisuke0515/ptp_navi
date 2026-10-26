"""Planar tag correction and bounded timestamp synchronization, independent of ROS."""
from collections import deque
from dataclasses import dataclass, fields
import math

NSEC = 1_000_000_000
MAX_SAMPLES = 1024


def wrap_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def center_pose(x, y, theta, forward=0.335, left=-0.010):
    yaw = wrap_pi(theta + math.pi / 2)
    return (x - math.cos(yaw) * forward + math.sin(yaw) * left,
            y - math.sin(yaw) * forward - math.cos(yaw) * left, yaw)


@dataclass(frozen=True)
class Config:
    tag_offset_forward_m: float = 0.335
    tag_offset_left_m: float = -0.010
    position_timeout_sec: float = 0.5
    heading_timeout_sec: float = 0.5
    max_heading_time_diff_sec: float = 0.1
    heading_wait_timeout_sec: float = 0.1
    heading_buffer_duration_sec: float = 1.0

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if not math.isfinite(value) or (field.name.endswith('_sec') and value <= 0):
                raise ValueError(f'{field.name} must be finite and durations positive')
        if self.heading_buffer_duration_sec < (
                self.position_timeout_sec + self.max_heading_time_diff_sec):
            raise ValueError('heading buffer must cover position timeout + heading time difference')


@dataclass(frozen=True)
class Pose:
    stamp: int
    x: float
    y: float
    yaw: float


class PoseSynchronizer:
    def __init__(self, config=Config(), warn=lambda reason: None):
        self.config = config
        self.warn = warn
        self.headings = deque(maxlen=MAX_SAMPLES)
        self.pending = deque(maxlen=MAX_SAMPLES)
        self.last_stamp = {}
        self.last_now = None

    def _clock(self, now):
        if self.last_now is not None and now < self.last_now:
            self.headings.clear()
            self.pending.clear()
            self.last_stamp.clear()
            self.warn('ROS clock moved backwards; synchronization reset')
        self.last_now = now
        cutoff = now - int(self.config.heading_buffer_duration_sec * NSEC)
        while self.headings and self.headings[0][0] < cutoff:
            self.headings.popleft()

    def _accept(self, kind, stamp, now, valid, values, timeout):
        self._clock(now)
        if not valid or not all(math.isfinite(v) for v in values):
            self.pending.clear()
            if kind == 'heading':
                self.headings.clear()
            self.warn(f'{kind}: invalid input')
            return False
        if stamp > now or now - stamp > int(timeout * NSEC):
            self.warn(f'{kind}: future or stale timestamp')
            return False
        if stamp <= self.last_stamp.get(kind, -1):
            self.warn(f'{kind}: duplicate or out-of-order timestamp')
            return False
        self.last_stamp[kind] = stamp
        return True

    def heading(self, stamp, yaw, valid, now):
        if self._accept('heading', stamp, now, valid, (yaw,),
                        self.config.heading_timeout_sec):
            if len(self.headings) == MAX_SAMPLES:
                self.warn('heading buffer full; dropping oldest sample')
            self.headings.append((stamp, wrap_pi(yaw)))
        return self.poll(now)

    def position(self, stamp, x, y, valid, now):
        if self._accept('position', stamp, now, valid, (x, y),
                        self.config.position_timeout_sec):
            if len(self.pending) == MAX_SAMPLES:
                self.warn('position buffer full; dropping oldest sample')
            self.pending.append((stamp, x, y, now))
        return self.poll(now)

    def _yaw(self, stamp, now):
        previous = None
        limit = int(self.config.max_heading_time_diff_sec * NSEC)
        for t, yaw in self.headings:
            if now - t > int(self.config.heading_timeout_sec * NSEC):
                continue
            if t == stamp:
                return yaw
            if t > stamp:
                if previous is None:
                    return None
                t0, y0 = previous
                if stamp - t0 <= limit and t - stamp <= limit:
                    return wrap_pi(y0 + (stamp - t0) / (t - t0) * wrap_pi(yaw - y0))
                return None
            previous = (t, yaw)
        return None

    def poll(self, now):
        self._clock(now)
        poses = []
        remaining = deque(maxlen=MAX_SAMPLES)
        for stamp, x, y, arrived in self.pending:
            if (now - stamp > int(self.config.position_timeout_sec * NSEC)
                    or now - arrived > int(self.config.heading_wait_timeout_sec * NSEC)):
                self.warn('position expired while waiting for heading')
                continue
            yaw = self._yaw(stamp, now)
            if yaw is None:
                remaining.append((stamp, x, y, arrived))
                continue
            # Never publish an older pose after a newer one becomes available.
            if remaining:
                self.warn('dropping older unmatched positions')
                remaining.clear()
            cx, cy, psi = center_pose(x, y, yaw, self.config.tag_offset_forward_m,
                                     self.config.tag_offset_left_m)
            poses.append(Pose(stamp, cx, cy, psi))
        self.pending = remaining
        return poses
