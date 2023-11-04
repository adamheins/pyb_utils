"""The module provides basic quaternion operations."""
import numpy as np
from spatialmath.base import r2q
from spatialmath import UnitQuaternion as UQ


def _UQ(q, **kwargs):
    """Helper to build a UnitQuaternion from (x, y, z, w) vector representation."""
    v, s = q[:3], q[3]
    return UQ(v=v, s=s, **kwargs)


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
    return UQ(s=q[3], v=q[:3]).R


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
    # use r2q explicitly for now since spatialmath may silently fail to convert
    # the rotation (see
    # https://github.com/bdaiinstitute/spatialmath-python/pull/87)
    q = r2q(C, order="xyzs")
    return _UQ(q).vec_xyzs


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
        The quaternion representing the Hamilton product :math:`q_0\\otimes
        q_1`. This quaternion represents the compound rotation obtained by
        rotating by :math:`q_0` and then :math:`q_1`.
    """
    Q0 = _UQ(q0, norm=normalize)
    Q1 = _UQ(q1, norm=normalize)
    Q = Q0 @ Q1
    return Q.vec_xyzs


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


def quatx(angle):
    """Quaternion representing a rotation of ``angle`` about the x-axis.

    Parameters
    ----------
    angle : float
        The angle of rotation in radians.

    Returns
    -------
    :
        The quaternion :math:`(x, y, z, w)` representing the rotation.
    """
    return UQ.Rx(angle).vec_xyzs


def quaty(angle):
    """Quaternion representing a rotation of ``angle`` about the y-axis.

    Parameters
    ----------
    angle : float
        The angle of rotation in radians.

    Returns
    -------
    :
        The quaternion :math:`(x, y, z, w)` representing the rotation.
    """
    return UQ.Ry(angle).vec_xyzs


def quatz(angle):
    """Quaternion representing a rotation of ``angle`` about the z-axis.

    Parameters
    ----------
    angle : float
        The angle of rotation in radians.

    Returns
    -------
    :
        The quaternion :math:`(x, y, z, w)` representing the rotation.
    """
    return UQ.Rz(angle).vec_xyzs
