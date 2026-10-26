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
    min_determinant: float,
    height_differences: Tuple[float, float, float] = (0.0, 0.0, 0.0),
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
    if abs(determinant) <= min_determinant:
        return None

    x = (b1 * a22 - a12 * b2) / determinant
    y = (a11 * b2 - b1 * a21) / determinant
    return (x, y) if math.isfinite(x) and math.isfinite(y) else None

