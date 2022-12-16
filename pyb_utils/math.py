from spatialmath.base import q2r, r2q, qunit


QUAT_ORDER = "xyzs"


def quaternion_to_matrix(q):
    """Convert quaternion q to rotation matrix."""
    return q2r(q, order=QUAT_ORDER)


def matrix_to_quaternion(C):
    """Convert rotation matrix C to quaternion."""
    return r2q(C, order=QUAT_ORDER)


def quaternion_multiply(q0, q1, normalize=True):
    """Hamilton product of two quaternions."""
    order = "xyzs"
    if normalize:
        q0 = qunit(q0)
        q1 = qunit(q1)
    C0 = quaternion_to_matrix(q0)
    C1 = quaternion_to_matrix(q1)
    return matrix_to_quaternion(C0 @ C1)


def quaternion_rotate(q, r):
    """Rotate point r by rotation represented by quaternion q."""
    return quaternion_to_matrix(q) @ r
