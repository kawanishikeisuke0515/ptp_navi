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


def test_goal_hold_requires_fresh_updates_and_complete_latches():
    c = ready()
    c.receive(2., 0., 0., .4, .4)
    assert c.state == 'GOAL_CHECK' and c.tick(.45) == (0., 0., 0.)
    for i in range(1, 12):
        t = .4 + i * .2
        c.receive(2., 0., 0., t, t)
    assert c.state == 'COMPLETE'
    assert c.tick(100.) == (0., 0., 0.)
    c = ready()
    c.receive(2., 0., 0., .4, .4)
    c.tick(2.5)
    assert c.state == 'ERROR'


def test_goal_exit_resets_hold():
    c = ready()
    c.receive(2., 0., 0., .4, .4)
    c.receive(2.2, 0., 0., .6, .6)
    assert c.state == 'DRIVE' and c.hold_started is None
    assert c.tick(.61)[0] < 0


def test_coincident_goal_no_division():
    c = Controller(Config(0., 0., init_sample_count=1), 0.)
    c.receive(0., 0., 0., 0., 0.)
    assert c.state == 'GOAL_CHECK'
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
    assert c.a is None and c.hold_started is None
    c.receive(0., 0., 0., .3, .3)
    assert c.state == 'INIT' and c.reason == ''
    assert c.init_started == .3 and len(c.samples) == 1

    c = Controller(Config(0., 0., init_sample_count=1, goal_hold_sec=.2), 0.)
    c.receive(0., 0., 0., 0., 0.)
    assert c.a == (0., 0.) and c.hold_started == 0.
    c.receive(0., 0., 0., .2, .2)
    assert c.state == 'COMPLETE'
    c.fail('frame_mismatch')
    assert c.state == 'COMPLETE' and c.reason == ''


def test_obstacle_pause_preserves_path_and_requires_continuous_clear():
    c = ready()
    start = c.a
    c.receive_obstacle(True, .21)
    assert c.state == 'PAUSE' and c.hold_started is None
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


def test_pause_resume_uses_current_position_and_restarts_goal_hold():
    c = ready(goal_hold_sec=.4, obstacle_clear_sec=.2)
    c.receive(2., 0., 0., .3, .3)
    c.receive_obstacle(True, .31)
    assert c.state == 'PAUSE' and c.hold_started is None
    c.receive(2., 0., 0., .4, .4)
    c.receive_obstacle(False, .4)
    c.receive(2., 0., 0., .6, .6)
    c.receive_obstacle(False, .61)
    assert c.state == 'GOAL_CHECK' and c.hold_started == .61
    c.receive(2., 0., 0., .8, .8)
    assert c.state == 'GOAL_CHECK'
    c.receive(2., 0., 0., 1.02, 1.02)
    assert c.state == 'COMPLETE'
    c.receive_obstacle(True, 1.03)
    assert c.state == 'COMPLETE'


def test_obstacle_during_init_blocks_start_and_keeps_initial_average():
    c = Controller(Config(2., 0., init_sample_count=2), 0.)
    c.receive_obstacle(True, 0.)
    c.receive(0., 0., 0., .1, .1)
    c.receive(.2, 0., 0., .2, .2)
    assert c.state == 'PAUSE' and c.a == pytest.approx((.1, 0.))


@pytest.mark.parametrize('kwargs', [
    {'goal_x': math.nan}, {'control_rate_hz': 0.}, {'init_sample_count': 2.5},
    {'min_linear_speed_m_s': 1.}, {'goal_hold_sec': -1.},
])
def test_invalid_config(kwargs):
    params = dict(goal_x=2., goal_y=0.)
    params.update(kwargs)
    with pytest.raises(ValueError):
        Config(**params)
