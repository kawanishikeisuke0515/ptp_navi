"""Height-aware planar trilateration shared by UWB nodes."""

import math
from typing import Optional, Tuple


def trilaterate_2d(
    anchor_1: Tuple[float, float],
    anchor_2: Tuple[float, float],
    anchor_3: Tuple[float, float],
    distance_1: float,
    distance_2: float,
    distance_3: float,
    height_differences: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    use_least_squares: bool = False,
) -> Optional[Tuple[float, float]]:
    """Estimate XY using signed tag-minus-anchor height differences in metres."""
    distances = (distance_1, distance_2, distance_3)
    if len(height_differences) != 3:
        raise ValueError('height_differences must contain three values')
    if not all(math.isfinite(v) for v in (*distances, *height_differences)):
        return None
    if any(d < abs(h) for d, h in zip(distances, height_differences)):
        return None
    # Squared horizontal ranges avoid unnecessary square roots.
    r1, r2, r3 = (
        (d - abs(h)) * (d + abs(h))
        for d, h in zip(distances, height_differences)
    )

    x1, y1 = anchor_1
    x2, y2 = anchor_2
    x3, y3 = anchor_3

    a11 = 2.0 * (x2 - x1)
    a12 = 2.0 * (y2 - y1)
    a21 = 2.0 * (x3 - x1)
    a22 = 2.0 * (y3 - y1)
    b1 = (
        r1
        - r2
        - x1 ** 2
        + x2 ** 2
        - y1 ** 2
        + y2 ** 2
    )
    b2 = (
        r1
        - r3
        - x1 ** 2
        + x3 ** 2
        - y1 ** 2
        + y3 ** 2
    )

    determinant = a11 * a22 - a12 * a21
    if determinant == 0.0:
        return None

    x = (b1 * a22 - a12 * b2) / determinant
    y = (a11 * b2 - b1 * a21) / determinant
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    if use_least_squares:
        return _refine_position(
            (x, y), (anchor_1, anchor_2, anchor_3),
            distances, height_differences,
        )
    return x, y


def _refine_position(position, anchors, distances, heights):
    """Minimize squared 3D range residuals using damped Gauss-Newton.

    Only accept finite improvements; retain the linear estimate if no step
    improves it. Bound the work per sample and require no extra dependencies.
    """
    def cost(x, y):
        return sum(
            (math.hypot(x - ax, y - ay, h) - d) ** 2
            for (ax, ay), d, h in zip(anchors, distances, heights)
        )

    x, y = position
    current_cost = cost(x, y)
    damping = 1e-3
    for _ in range(40):
        a = c = damping
        b = gx = gy = 0.0
        for (ax, ay), d, h in zip(anchors, distances, heights):
            predicted = math.hypot(x - ax, y - ay, h)
            if predicted <= 1e-12:
                continue
            jx, jy = (x - ax) / predicted, (y - ay) / predicted
            residual = predicted - d
            a += jx * jx
            b += jx * jy
            c += jy * jy
            gx += jx * residual
            gy += jy * residual
        if math.hypot(gx, gy) <= 1e-10:
            break
        determinant = a * c - b * b
        if determinant <= 0.0 or not math.isfinite(determinant):
            break
        dx = (c * gx - b * gy) / determinant
        dy = (a * gy - b * gx) / determinant
        candidate_x, candidate_y = x - dx, y - dy
        if not all(math.isfinite(v) for v in (candidate_x, candidate_y)):
            break
        candidate_cost = cost(candidate_x, candidate_y)
        if math.isfinite(candidate_cost) and candidate_cost < current_cost:
            x, y = candidate_x, candidate_y
            current_cost = candidate_cost
            damping = max(damping / 3.0, 1e-12)
            if math.hypot(dx, dy) <= 1e-8:
                break
        else:
            damping *= 10.0
    return x, y

