import math


def frame_to_pose(frame):
    """Convert a COMPAS Frame to a UR pose [x, y, z, rx, ry, rz]."""
    xaxis = frame.xaxis
    yaxis = frame.yaxis
    zaxis = frame.zaxis

    # Rotation matrix whose columns are the frame axes.
    r00, r01, r02 = xaxis.x, yaxis.x, zaxis.x
    r10, r11, r12 = xaxis.y, yaxis.y, zaxis.y
    r20, r21, r22 = xaxis.z, yaxis.z, zaxis.z

    # Convert the rotation matrix to a unit quaternion.
    trace = r00 + r11 + r22
    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        qw = 0.25 * scale
        qx = (r21 - r12) / scale
        qy = (r02 - r20) / scale
        qz = (r10 - r01) / scale
    elif r00 > r11 and r00 > r22:
        scale = 2.0 * math.sqrt(1.0 + r00 - r11 - r22)
        qw = (r21 - r12) / scale
        qx = 0.25 * scale
        qy = (r01 + r10) / scale
        qz = (r02 + r20) / scale
    elif r11 > r22:
        scale = 2.0 * math.sqrt(1.0 + r11 - r00 - r22)
        qw = (r02 - r20) / scale
        qx = (r01 + r10) / scale
        qy = 0.25 * scale
        qz = (r12 + r21) / scale
    else:
        scale = 2.0 * math.sqrt(1.0 + r22 - r00 - r11)
        qw = (r10 - r01) / scale
        qx = (r02 + r20) / scale
        qy = (r12 + r21) / scale
        qz = 0.25 * scale

    quaternion_length = math.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    qw /= quaternion_length
    qx /= quaternion_length
    qy /= quaternion_length
    qz /= quaternion_length

    # q and -q describe the same rotation. Choose an angle in [0, pi].
    if qw < 0.0:
        qw, qx, qy, qz = -qw, -qx, -qy, -qz

    vector_length = math.sqrt(qx**2 + qy**2 + qz**2)
    if vector_length < 1e-12:
        rx = ry = rz = 0.0
    else:
        angle = 2.0 * math.atan2(vector_length, qw)
        factor = angle / vector_length
        rx, ry, rz = qx * factor, qy * factor, qz * factor

    return [frame.point.x, frame.point.y, frame.point.z, rx, ry, rz]
