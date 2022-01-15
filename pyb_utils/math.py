from liegroups import SO3


QUAT_ORDER = "xyzw"


def quaternion_rotate(quat, point):
    """Rotate a point using a unit quaternion."""
    return SO3.from_quaternion(quat, ordering=QUAT_ORDER).dot(point)


def quaternion_multiply(quat1, quat2):
    """Hamilton product of two quaternions stored in xyzw order."""
    C1 = SO3.from_quaternion(quat1, ordering=QUAT_ORDER)
    C2 = SO3.from_quaternion(quat2, ordering=QUAT_ORDER)
    return C1.dot(C2).to_quaterion(ordering=QUAT_ORDER)
