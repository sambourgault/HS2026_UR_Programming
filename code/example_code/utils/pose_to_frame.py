import math

from compas.geometry import Frame, Point, Vector


def pose_to_frame(pose):
    """Convert a UR pose [x, y, z, rx, ry, rz] to a COMPAS Frame."""
    if len(pose) != 6:
        raise ValueError("A UR pose must contain exactly 6 values")

    x, y, z, rx, ry, rz = pose
    angle = math.sqrt(rx**2 + ry**2 + rz**2)

    if angle < 1e-12:
        return Frame(Point(x, y, z), Vector(1, 0, 0), Vector(0, 1, 0))

    ax, ay, az = rx / angle, ry / angle, rz / angle
    cosine = math.cos(angle)
    sine = math.sin(angle)
    one_minus_cosine = 1.0 - cosine

    xaxis = Vector(
        ax * ax * one_minus_cosine + cosine,
        ay * ax * one_minus_cosine + az * sine,
        az * ax * one_minus_cosine - ay * sine,
    )
    yaxis = Vector(
        ax * ay * one_minus_cosine - az * sine,
        ay * ay * one_minus_cosine + cosine,
        az * ay * one_minus_cosine + ax * sine,
    )

    return Frame(Point(x, y, z), xaxis, yaxis)
