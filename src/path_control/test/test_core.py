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


@pytest.mark.parametrize('kwargs', [
    {'goal_x': math.nan}, {'control_rate_hz': 0.}, {'init_sample_count': 2.5},
    {'min_linear_speed_m_s': 1.}, {'goal_hold_sec': -1.},
])
def test_invalid_config(kwargs):
    params = dict(goal_x=2., goal_y=0.)
    params.update(kwargs)
    with pytest.raises(ValueError):
        Config(**params)
