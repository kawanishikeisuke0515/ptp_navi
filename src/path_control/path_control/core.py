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
    obstacle_clear_sec: float = 1.0
    obstacle_timeout_sec: float = 0.6

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
        self.obstacle_detected = False
        self.obstacle_received = None
        self.obstacle_clear_started = None

    @property
    def timeout(self):
        return min(self.c.position_timeout_sec, self.c.heading_timeout_sec)

    def fail(self, reason):
        """Accept an error notification, including errors from the ROS adapter."""
        self._transition_to(*self._next_state('failure', reason=reason))

    def _next_state(self, event, now=None, reason=''):
        """Decide the next state without modifying controller data.

        Events: failure, timer, valid_pose (before storage), pose (after storage), obstacle.
        Timer checks precede input replacement so a late pose cannot hide expiry.
        """
        if self.state == 'COMPLETE':
            return self.state, self.reason
        if event == 'failure':
            return 'ERROR', reason
        # Common error conditions take priority over state-specific transitions.
        if event == 'timer' and self.received is not None and (
                now - self.received >= self.timeout or now - self.stamp >= self.timeout):
            return 'ERROR', 'pose_timeout'

        # Monitoring begins with the first obstacle message. Once started,
        # missing messages block motion just like an active detection.
        obstacle_stale = (now is not None and self.obstacle_received is not None
                          and now - self.obstacle_received >= self.c.obstacle_timeout_sec)
        clear_pending = (now is not None and self.obstacle_clear_started is not None
                         and now - self.obstacle_clear_started < self.c.obstacle_clear_sec)
        obstacle_blocked = self.obstacle_detected or obstacle_stale or clear_pending
        obstacle_reason = ('obstacle_timeout' if obstacle_stale else
                           'obstacle_detected' if self.obstacle_detected else 'obstacle_clearing')

        if self.state == 'INIT':
            if event == 'timer' and now - self.init_started >= self.c.init_timeout_sec:
                return 'ERROR', 'init_timeout'
            if event == 'pose' and len(self.samples) >= self.c.init_sample_count:
                if obstacle_blocked:
                    return 'PAUSE', obstacle_reason
                distance = math.hypot(self.c.goal_x - self.pose[0], self.c.goal_y - self.pose[1])
                # Initialization may finish directly inside the goal region.
                if distance <= self.c.goal_tolerance_m:
                    return 'GOAL_CHECK', self.reason
                return 'DRIVE', self.reason

        elif self.state == 'DRIVE':
            if event == 'timer' and math.hypot(
                    self.c.goal_x - self.a[0], self.c.goal_y - self.a[1]) < 1e-9:
                return 'ERROR', 'degenerate_path'
            if obstacle_blocked:
                return 'PAUSE', obstacle_reason
            if event == 'pose':
                distance = math.hypot(self.c.goal_x - self.pose[0], self.c.goal_y - self.pose[1])
                if distance <= self.c.goal_tolerance_m:
                    return 'GOAL_CHECK', self.reason

        elif self.state == 'GOAL_CHECK':
            if obstacle_blocked:
                return 'PAUSE', obstacle_reason
            if event == 'pose':
                distance = math.hypot(self.c.goal_x - self.pose[0], self.c.goal_y - self.pose[1])
                if distance > self.c.goal_tolerance_m:
                    return 'DRIVE', self.reason
                if self.stamp - self.hold_started >= self.c.goal_hold_sec:
                    return 'COMPLETE', self.reason

        elif self.state == 'PAUSE':
            if obstacle_blocked:
                return 'PAUSE', obstacle_reason
            # Resume only on a fresh clear message, never on elapsed time alone.
            if (event == 'obstacle' and self.obstacle_clear_started is not None
                    and now - self.obstacle_clear_started >= self.c.obstacle_clear_sec):
                distance = math.hypot(self.c.goal_x - self.pose[0], self.c.goal_y - self.pose[1])
                if distance <= self.c.goal_tolerance_m:
                    return 'GOAL_CHECK', ''
                return 'DRIVE', ''

        elif self.state == 'ERROR':
            if event == 'valid_pose':
                return 'INIT', ''

        # COMPLETE is handled first because it ignores even common errors.
        return self.state, self.reason

    def _transition_to(self, next_state, reason, now=None):
        """Apply a decision and maintain the data associated with each state."""
        previous = self.state
        self.state, self.reason = next_state, reason
        # Repeated failures must also clear initialization data.
        if next_state == 'ERROR':
            self.samples.clear()
            self.a = None
            self.hold_started = None
        elif next_state != previous:
            if next_state == 'INIT':
                self.init_started = now
                self.samples.clear()
                self.a = None
            if previous == 'INIT' and next_state in ('DRIVE', 'GOAL_CHECK', 'PAUSE'):
                self.a = tuple(sum(p[i] for p in self.samples) / len(self.samples)
                               for i in (0, 1))
            if next_state == 'GOAL_CHECK':
                # After PAUSE, do not count time preceding obstacle clearance.
                self.hold_started = now if previous == 'PAUSE' else self.stamp
            elif next_state in ('DRIVE', 'PAUSE'):
                self.hold_started = None

    def _clock(self, now):
        if now < self.last_now:
            self.fail('clock_backwards')
            self.stamp = None
            self.received = None
            self.obstacle_received = None
            self.obstacle_detected = True
            self.obstacle_clear_started = None
        self.last_now = now

    def tick(self, now):
        self._clock(now)
        self._transition_to(*self._next_state('timer', now), now=now)
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
        self._transition_to(*self._next_state('valid_pose', now), now=now)
        self.pose = (x, y, yaw)
        self.stamp, self.received = stamp, now
        if self.state == 'INIT':
            self.samples.append((x, y))
        self._transition_to(*self._next_state('pose', now), now=now)
        return True

    def receive_obstacle(self, detected, now):
        """Store detection and require continuously fresh false messages to resume."""
        self.tick(now)  # Check gaps before replacing the previous receipt time.
        if self.state == 'COMPLETE':
            return
        gap = (self.obstacle_received is None
               or now - self.obstacle_received >= self.c.obstacle_timeout_sec)
        if detected:
            self.obstacle_clear_started = None
        elif gap or self.obstacle_detected or self.obstacle_clear_started is None:
            self.obstacle_clear_started = now
        self.obstacle_detected = detected
        self.obstacle_received = now
        self._transition_to(*self._next_state('obstacle', now), now=now)
