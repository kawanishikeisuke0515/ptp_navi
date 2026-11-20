import math
import pytest
from path_control.core import Config, Controller, track


def ready(**kwargs):
    c = Controller(Config(goal_x=2., goal_y=0., init_sample_count=2, **kwargs), 0.)
    c.receive(0., 0., 0., 0., 0.)
    c.receive(0., 0., 0., .2, .2)
    assert c.state == 'DRIVE'
    return c


def test_tracking_geometry_limits_and_overshoot():
    c = Config(2., 0.)
    assert track((0, 0), (2, 0), (0, 0, 0), c) == pytest.approx((.3, 0, 0))
    vx, vy, omega = track((0, 0), (2, 0), (0, 2, 0), c)
    assert vx > 0 and vy < 0 and math.hypot(vx, vy) == pytest.approx(.5)
    assert track((0, 0), (2, 0), (2.2, 0, 0), c)[0] == pytest.approx(-.3)
    vx, vy, omega = track((0, 0), (2, 0), (0, 0, math.pi/2), c)
    assert (vx, vy, omega) == pytest.approx((0, -.3, -.3))
    c = Config(-2., -.01)
    assert track((0, 0), (-2, -.01), (0, 0, math.pi-.01), c)[2] > 0
    c = Config(2., 0., path_speed_m_s=.01)
    assert track((0, 0), (2, 0), (0, 0, 0), c)[0] == pytest.approx(.05)


def test_init_averages_only_new_samples():
    c = Controller(Config(2., 0., init_sample_count=2), 0.)
    c.receive(0., 0., 0., 0., 0.)
    for t in [.05, .1, .15]:
        assert c.tick(t) == (0., 0., 0.)
    assert len(c.samples) == 1
    c.receive(.2, 0., 0., .2, .2)
    assert c.a == pytest.approx((.1, 0.))
    assert c.tick(.21)[0] > 0


def test_timeout_stops_and_requires_new_stamp_then_reinitializes():
    c = ready()
    assert c.tick(.81) == (0., 0., 0.)
    assert c.state == 'ERROR'
    assert c.a is None
    c.receive(0., 0., 0., .2, .82)
    assert c.state == 'ERROR'
    c.receive(1., 0., 0., .9, .9)
    assert c.state == 'INIT' and len(c.samples) == 1
    c.receive(1., 0., 0., 1.1, 1.1)
    assert c.a == (1., 0.) and c.state == 'DRIVE'


def test_init_timeout_and_late_callback():
    c = Controller(Config(2., 0.), 0.)
    assert c.tick(10.) == (0., 0., 0.)
    assert c.reason == 'init_timeout'
    c.receive(0., 0., 0., 10.1, 10.1)
    assert c.state == 'INIT' and len(c.samples) == 1
    c = ready()
    c.receive(1., 0., 0., 1., 1.)
    assert c.state == 'INIT'  # Gap must not be hidden by the new input.


@pytest.mark.parametrize('offset, complete', [(0., True), (.125, True), (.126, False)])
def test_single_fresh_pose_completes_at_tolerance_boundary(offset, complete):
    c = ready(goal_tolerance_m=.125)
    c.receive(2., offset, 1., .4, .4)
    assert c.state == ('COMPLETE' if complete else 'DRIVE')
    if complete:
        assert c.tick(.41) == (0., 0., 0.)
        assert not c.receive(3., 0., 0., .5, .5)
        assert c.tick(100.) == (0., 0., 0.)
        assert c.state == 'COMPLETE'


def test_overshoot_outside_tolerance_returns_toward_goal():
    c = ready()
    c.receive(2.2, 0., 0., .4, .4)
    assert c.state == 'DRIVE'
    assert c.tick(.41)[0] < 0
    c.receive(2., 0., 0., .5, .5)
    assert c.state == 'COMPLETE'


@pytest.mark.parametrize('stamp, now', [(.2, .3), (.1, .3), (.9, .3), (.3, 1.)])
def test_invalid_or_late_goal_pose_cannot_complete(stamp, now):
    c = ready()
    c.receive(2., 0., 0., stamp, now)
    assert c.state != 'COMPLETE'
    assert c.tick(now) == (0., 0., 0.)


def test_coincident_goal_no_division():
    c = Controller(Config(0., 0., init_sample_count=1), 0.)
    c.receive(0., 0., 0., 0., 0.)
    assert c.state == 'COMPLETE'
    assert c.tick(.1) == (0., 0., 0.)


@pytest.mark.parametrize('sample', [
    (math.nan, 0., 0., .3), (0., 0., 0., .2), (0., 0., 0., 5.),
])
def test_invalid_data_stops(sample):
    c = ready()
    c.receive(*sample, .3)
    assert c.state == 'ERROR' and c.tick(.31) == (0., 0., 0.)


def test_clock_backwards():
    c = ready()
    assert c.tick(.1) == (0., 0., 0.)
    assert c.reason == 'clock_backwards'


def test_next_state_decision_does_not_apply_transition():
    c = ready()
    assert c._next_state('timer', .81) == ('ERROR', 'pose_timeout')
    assert c.state == 'DRIVE'
    assert c.a == (0., 0.)
    assert len(c.samples) == 2
    c.tick(.81)
    assert c.state == 'ERROR'
    assert c.a is None and c.samples == []


def test_external_failure_recovery_and_terminal_state():
    c = ready()
    c.fail('frame_mismatch')
    assert c.state == 'ERROR' and c.reason == 'frame_mismatch'
    assert c.a is None
    c.receive(0., 0., 0., .3, .3)
    assert c.state == 'INIT' and c.reason == ''
    assert c.init_started == .3 and len(c.samples) == 1

    c = Controller(Config(0., 0., init_sample_count=1), 0.)
    c.receive(0., 0., 0., 0., 0.)
    assert c.a == (0., 0.)
    c.receive(0., 0., 0., .2, .2)
    assert c.state == 'COMPLETE'
    c.fail('frame_mismatch')
    assert c.state == 'COMPLETE' and c.reason == ''


def test_obstacle_pause_preserves_path_and_requires_continuous_clear():
    c = ready()
    start = c.a
    c.receive_obstacle(True, .21)
    assert c.state == 'PAUSE'
    assert c.tick(.22) == (0., 0., 0.)
    for t in (.3, .5, .7):
        c.receive(1., 0., 0., t, t)
        c.receive_obstacle(False, t)
        assert c.state == 'PAUSE'
    c.receive_obstacle(True, .71)  # Detection restarts the clear interval.
    for t in (.8, 1., 1.2, 1.4, 1.6, 1.8):
        c.receive(1., 0., 0., t, t)
        c.receive_obstacle(False, t)
    assert c.state == 'DRIVE' and c.a == start
    assert c.tick(1.81)[0] > 0


def test_obstacle_gap_does_not_release_pause():
    c = ready()
    c.receive_obstacle(True, .21)
    c.receive_obstacle(False, .3)
    for t in (.4, .6, .8, 1., 1.2, 1.4):
        c.receive(1., 0., 0., t, t)
    assert c.state == 'PAUSE' and c.reason == 'obstacle_timeout'
    c.receive_obstacle(False, 1.4)
    assert c.obstacle_clear_started == 1.4 and c.state == 'PAUSE'


def test_obstacle_monitor_timeout_stops_drive_and_pose_timeout_is_error():
    c = ready()
    c.receive_obstacle(False, .21)
    for t in (.4, .6, .82):
        c.receive(1., 0., 0., t, t)
    assert c.state == 'PAUSE' and c.reason == 'obstacle_timeout'
    assert c.tick(1.43) == (0., 0., 0.)
    assert c.state == 'ERROR' and c.reason == 'pose_timeout'


@pytest.mark.parametrize('mode', ['detected', 'stale', 'clearing'])
def test_goal_during_obstacle_pause_completes(mode):
    c = ready()
    if mode == 'detected':
        c.receive_obstacle(True, .21)
        arrival = .3
    elif mode == 'clearing':
        c.receive_obstacle(False, .21)
        arrival = .3
    else:
        c.receive_obstacle(False, .21)
        c.receive(1., 0., 0., .5, .5)
        c.tick(.82)
        arrival = .83
    assert c.state == 'PAUSE'
    c.receive(2., 0., 0., arrival, arrival)
    assert c.state == 'COMPLETE' and c.reason == ''
    c.receive_obstacle(True, arrival + .01)
    assert c.tick(arrival + .02) == (0., 0., 0.)
    assert c.state == 'COMPLETE'


@pytest.mark.parametrize('positions, expected', [((0., 2.), 'COMPLETE'), ((4., 0.), 'PAUSE')])
def test_init_arrival_uses_latest_pose_not_average_despite_obstacle(positions, expected):
    c = Controller(Config(2., 0., init_sample_count=2), 0.)
    c.receive_obstacle(True, 0.)
    c.receive(positions[0], 0., 0., .1, .1)
    assert c.state == 'INIT'
    c.receive(positions[1], 0., 0., .2, .2)
    assert c.state == expected
    assert c.a == (sum(positions) / 2, 0.)
    assert c.tick(.21) == (0., 0., 0.)


def test_obstacle_during_init_blocks_start_and_keeps_initial_average():
    c = Controller(Config(2., 0., init_sample_count=2), 0.)
    c.receive_obstacle(True, 0.)
    c.receive(0., 0., 0., .1, .1)
    c.receive(.2, 0., 0., .2, .2)
    assert c.state == 'PAUSE' and c.a == pytest.approx((.1, 0.))


@pytest.mark.parametrize('kwargs', [
    {'goal_x': math.nan}, {'control_rate_hz': 0.}, {'init_sample_count': 2.5},
    {'min_linear_speed_m_s': 1.}, {'goal_tolerance_m': -1.},
])
def test_invalid_config(kwargs):
    params = dict(goal_x=2., goal_y=0.)
    params.update(kwargs)
    with pytest.raises(ValueError):
        Config(**params)
