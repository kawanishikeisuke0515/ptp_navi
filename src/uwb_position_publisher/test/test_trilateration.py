"""Geometric regression tests independent of ROS."""
import math

import pytest

from uwb_position_publisher.trilateration import trilaterate_2d


ANCHORS = ((0.0, 0.0), (4.0, 0.0), (0.0, 4.0))


def ranges(point, heights):
    return tuple(math.sqrt((point[0]-x)**2 + (point[1]-y)**2 + h**2)
                 for (x, y), h in zip(ANCHORS, heights))


@pytest.mark.parametrize('heights', [
    (0.0, 0.0, 0.0), (1.5, 1.5, 1.5), (-1.5, -1.5, -1.5),
    (0.5, -1.0, 2.0),
])
def test_known_position(heights):
    expected = (1.0, 2.0)
    assert trilaterate_2d(*ANCHORS, *ranges(expected, heights), 1e-9,
                          heights) == pytest.approx(expected)


def test_legacy_call():
    assert trilaterate_2d(*ANCHORS, *ranges((1, 2), (0, 0, 0)),
                          1e-9) == pytest.approx((1, 2))


def test_directly_above_anchor():
    heights = (2.0, 1.0, -0.5)
    assert trilaterate_2d(*ANCHORS, *ranges((0, 0), heights), 1e-9,
                          heights) == pytest.approx((0, 0))


@pytest.mark.parametrize('distance,height', [
    (0.9, 1.0), (-1.0, 0.0), (math.nan, 0.0), (math.inf, 0.0),
    (1.0, math.nan), (1.0, math.inf),
])
def test_invalid_range_or_height(distance, height):
    assert trilaterate_2d(*ANCHORS, distance, 3.0, 3.0, 1e-9,
                          (height, 0.0, 0.0)) is None


def test_collinear_anchors():
    assert trilaterate_2d((0, 0), (1, 0), (2, 0), 2, 2, 2,
                          1e-9, (1, 1, 1)) is None
