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
@pytest.mark.parametrize('use_least_squares', [False, True])
def test_known_position(heights, use_least_squares):
    expected = (1.0, 2.0)
    assert trilaterate_2d(*ANCHORS, *ranges(expected, heights),
                          heights, use_least_squares) == pytest.approx(expected)


def test_legacy_call():
    assert trilaterate_2d(*ANCHORS, *ranges((1, 2), (0, 0, 0)),
                          ) == pytest.approx((1, 2))


@pytest.mark.parametrize('use_least_squares', [False, True])
def test_directly_above_anchor(use_least_squares):
    heights = (2.0, 1.0, -0.5)
    assert trilaterate_2d(*ANCHORS, *ranges((0, 0), heights),
                          heights, use_least_squares) == pytest.approx((0, 0))


@pytest.mark.parametrize('distance,height', [
    (0.9, 1.0), (-1.0, 0.0), (math.nan, 0.0), (math.inf, 0.0),
    (1.0, math.nan), (1.0, math.inf),
])
@pytest.mark.parametrize('use_least_squares', [False, True])
def test_invalid_range_or_height(distance, height, use_least_squares):
    assert trilaterate_2d(*ANCHORS, distance, 3.0, 3.0,
                          (height, 0.0, 0.0), use_least_squares) is None


@pytest.mark.parametrize('use_least_squares', [False, True])
def test_collinear_anchors(use_least_squares):
    assert trilaterate_2d((0, 0), (1, 0), (2, 0), 2, 2, 2,
                          (1, 1, 1), use_least_squares) is None


@pytest.mark.parametrize('heights', [(0, 0, 0), (0.5, -1.0, 2.0)])
def test_least_squares_reduces_noisy_range_error(heights):
    distances = tuple(d + noise for d, noise in zip(
        ranges((1, 2), heights), (0.3, -0.1, 0.2)))
    linear = trilaterate_2d(*ANCHORS, *distances, heights)
    refined = trilaterate_2d(
        *ANCHORS, *distances, heights, use_least_squares=True)

    def cost(point):
        return sum((predicted - measured)**2 for predicted, measured in
                   zip(ranges(point, heights), distances))

    assert cost(refined) < cost(linear)
    # At a local least-squares solution, the cost gradient vanishes.
    epsilon = 1e-5
    for axis in range(2):
        plus, minus = list(refined), list(refined)
        plus[axis] += epsilon
        minus[axis] -= epsilon
        gradient = (cost(plus) - cost(minus)) / (2 * epsilon)
        assert abs(gradient) < 1e-6


def test_explicitly_disabled_preserves_linear_result():
    distances = (2.5, 3.8, 2.4)
    assert trilaterate_2d(*ANCHORS, *distances) == trilaterate_2d(
        *ANCHORS, *distances, use_least_squares=False)
