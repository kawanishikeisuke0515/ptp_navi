"""Rigid-body origin to robot center conversion, in the original mocap frame."""
import math


def center_position(position, quaternion, offset):
    """offset is center -> rigid-body origin, expressed in rigid-body XYZ axes."""
    if not all(math.isfinite(v) for v in (*position, *quaternion, *offset)):
        raise ValueError('Pose and offset must be finite')
    norm = math.hypot(*quaternion)
    if abs(norm - 1.0) > 0.01:
        raise ValueError('Quaternion must have unit norm')
    x, y, z, w = (v / norm for v in quaternion)
    ox, oy, oz = offset
    rotated = (
        (1-2*(y*y+z*z))*ox + 2*(x*y-z*w)*oy + 2*(x*z+y*w)*oz,
        2*(x*y+z*w)*ox + (1-2*(x*x+z*z))*oy + 2*(y*z-x*w)*oz,
        2*(x*z-y*w)*ox + 2*(y*z+x*w)*oy + (1-2*(x*x+y*y))*oz,
    )
    return tuple(p - d for p, d in zip(position, rotated))
