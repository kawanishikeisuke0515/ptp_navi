import math

import pytest

from uwb_robot_pose_publisher.pose_core import (
    Config, MAX_SAMPLES, PoseSynchronizer, center_pose,
)


def test_geometry():
    assert center_pose(1, 2, 0) == pytest.approx((.990, 1.665, math.pi / 2))
    assert center_pose(1, 2, math.pi / 2) == pytest.approx((1.335, 1.990, -math.pi))
    assert center_pose(1, 2, .4, 0, 0)[:2] == (1, 2)
    for degrees in range(-180, 181):
        theta = math.radians(degrees)
        psi = theta + math.pi / 2
        tx = 3 + .335 * math.cos(psi) + .010 * math.sin(psi)
        ty = 4 + .335 * math.sin(psi) - .010 * math.cos(psi)
        assert center_pose(tx, ty, theta)[:2] == pytest.approx((3, 4))


def test_interpolation_wrap_and_once_only():
    s = PoseSynchronizer()
    s.heading(1_000_000_000, math.radians(179), True, 1_000_000_000)
    assert not s.position(1_050_000_000, 1, 2, True, 1_050_000_000)
    poses = s.heading(1_100_000_000, math.radians(-179), True, 1_100_000_000)
    assert len(poses) == 1
    assert poses[0].stamp == 1_050_000_000
    assert poses[0].yaw == pytest.approx(-math.pi / 2)
    assert not s.poll(1_100_000_000)
    assert not s.heading(1_150_000_000, 0, True, 1_150_000_000)


def test_exact_stamp_and_duplicate():
    s = PoseSynchronizer()
    s.heading(100, 0, True, 100)
    assert len(s.position(100, 1, 2, True, 100)) == 1
    assert not s.position(100, 1, 2, True, 100)
    assert not s.position(99, 1, 2, True, 100)


@pytest.mark.parametrize('valid,x', [(False, 1), (True, math.nan), (True, math.inf)])
def test_invalid_position_clears_pending(valid, x):
    s = PoseSynchronizer()
    s.position(100, 1, 2, True, 100)
    s.position(101, x, 2, valid, 101)
    assert not s.pending


def test_invalid_heading_clears_interpolation_history():
    s = PoseSynchronizer()
    s.heading(100, 0, True, 100)
    s.position(110, 1, 2, True, 110)
    s.heading(120, 0, False, 120)
    assert not s.headings and not s.pending
    s.position(130, 1, 2, True, 130)
    assert not s.heading(140, 0, True, 140)


def test_expiration_future_stale_and_gap():
    s = PoseSynchronizer()
    assert not s.position(2_000_000_000, 1, 2, True, 1_000_000_000)
    assert not s.position(1, 1, 2, True, 1_000_000_000)
    s.heading(1_000_000_000, 0, True, 1_000_000_000)
    s.position(1_200_000_000, 1, 2, True, 1_200_000_000)
    assert not s.heading(1_250_000_000, 0, True, 1_250_000_000)
    assert not s.poll(1_300_000_001)
    assert not s.pending


def test_clock_reset_and_bounded_queue():
    s = PoseSynchronizer()
    s.heading(100, 0, True, 100)
    s.position(110, 1, 2, True, 110)
    s.poll(0)
    assert not s.headings and not s.pending and not s.last_stamp
    for t in range(MAX_SAMPLES + 10):
        s.position(t, 1, 2, True, t)
    assert len(s.pending) == MAX_SAMPLES


@pytest.mark.parametrize('kwargs', [
    {'heading_timeout_sec': 0}, {'tag_offset_left_m': math.nan},
    {'heading_buffer_duration_sec': .1}, {'position_timeout_sec': math.inf},
])
def test_bad_config(kwargs):
    with pytest.raises(ValueError):
        Config(**kwargs)
