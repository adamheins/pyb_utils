"""The module provides basic quaternion operations."""
from spatialmath.base import q2r, r2q, qunit


QUAT_ORDER = "xyzs"


def quaternion_to_matrix(q):
    """Convert a quaternion to a rotation matrix.

    Parameters
    ----------
    q : iterable
        A quaternion :math:`(x, y, z, w)`.

    Returns
    -------
    :
        The :math:`3\\times3` rotation matrix representing the same rotation
        matrix as ``q``.
    """
    return q2r(q, order=QUAT_ORDER)


def matrix_to_quaternion(C):
    """Convert a rotation matrix to a quaternion.

    Parameters
    ----------
    C :
        A :math:`3\\times3` rotation matrix.

    Returns
    -------
    :
        The quaternion :math:`(x, y, z, w)` representing the same rotation as
        ``C``.
    """
    return r2q(C, order=QUAT_ORDER)


def quaternion_multiply(q0, q1, normalize=True):
    """The Hamilton product of two quaternions.

    Parameters
    ----------
    q0 : iterable
        The first quaternion, in :math:`(x, y, z, w)` order.
    q1 : iterable
        The second quaternion, in :math:`(x, y, z, w)` order.
    normalize : bool
        Normalize the quaterions to ensure they are unit.

    Returns
    -------
    :
        The quaternion representing the Hamilton product :math:`q_0\otimes
        q_1`. This quaternion represents the compound rotation obtained by
        rotating by :math:`q_0` and then :math:`q_1`.
    """
    if normalize:
        q0 = qunit(q0)
        q1 = qunit(q1)
    C0 = quaternion_to_matrix(q0)
    C1 = quaternion_to_matrix(q1)
    return matrix_to_quaternion(C0 @ C1)


def quaternion_rotate(q, r):
    """Rotate a point by the rotation represented by a quaternion.

    Parameters
    ----------
    q : iterable
        The quaternion :math:`(x, y, z, w)` representing the rotation.
    r : iterable
        The point to rotate.

    Returns
    -------
    :
        The rotated point.
    """
    return quaternion_to_matrix(q) @ r
