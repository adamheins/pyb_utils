import numpy as np
import pybullet as pyb

from .named_tuples import getContactPoints


def _right_handed(point):
    """Returns True if the coordinate system of the contact point is
    right-handed, False otherwise.

    See https://math.stackexchange.com/q/327841.
    """
    v1 = point.contactNormalOnB
    v2 = point.lateralFrictionDir1
    v3 = point.lateralFrictionDir2
    return np.cross(v1, v2) @ v3 > 0


def get_point_contact_wrench(point, origin=None):
    """Compute the contact wrench at a contact point.

    The wrench is the force and torque acting on body A due to contact with
    body B at contact point ``point``. The wrench is aligned with the world
    frame.

    Parameters
    ----------
    point : ContactPoint
        The contact point, as returned by
        :func:`pyb_utils.named_tuples.getContactPoints`.
    origin :
        The origin point about which to compute the torque. If ``None`` (the
        default), then the torque is computed about the world frame origin.

    Returns
    -------
    :
        A tuple ``(force, torque)`` representing the contact wrench on body A.
    """
    if origin is None:
        # world origin
        origin = np.zeros(3)
    origin = np.array(origin)
    assert origin.shape == (3,), "Origin must be of length 3."

    nf = point.normalForce * np.array(point.contactNormalOnB)
    ff1 = point.lateralFriction1 * np.array(point.lateralFrictionDir1)
    ff2 = point.lateralFriction2 * np.array(point.lateralFrictionDir2)

    # lateral friction forces are always the same reported as the same
    # regardless of the order of body A and B, but we can get the correct
    # order by ensuring they make a right-handed frame together with the
    # contact normal (which does depend on the order of A and B)
    if not _right_handed(point):
        ff1 *= -1
        ff2 *= -1
    force = nf + ff1 + ff2
    torque = np.cross(point.positionOnA - origin, force)
    return force, torque


def get_points_contact_wrench(points, origin=None):
    """Compute the contact wrench resulting from a set of contact points.

    The wrench is the force and torque acting on body A due to contact with
    body B at contact points ``points``. The wrench is aligned with the world
    frame.

    Parameters
    ----------
    points :
        List of contact points, as returned by
        :func:`pyb_utils.named_tuples.getContactPoints`.
    origin :
        The origin point about which to compute the torque. If ``None`` (the
        default), then the torque is computed about the world frame origin.

    Returns
    -------
    :
        A tuple ``(force, torque)`` representing the contact wrench.
    """
    force = np.zeros(3)
    torque = np.zeros(3)
    for point in points:
        f, τ = get_point_contact_wrench(point, origin=origin)
        force += f
        torque += τ
    return force, torque


def get_total_contact_wrench(
    bodyA,
    bodyB,
    linkIndexA=-2,
    linkIndexB=-2,
    origin=None,
    max_contacts=None,
    physicsClientId=0,
):
    """Compute the contact wrench resulting from a set of contact points.

    The wrench is the force and torque acting on body A due to contact with
    body B. The wrench is aligned with the world frame.

    If :func:`pyb_utils.named_tuples.getContactPoints` has already been called,
    you can use the :func:`get_points_contact_wrench` function instead.

    Parameters
    ----------
    bodyA : int
        The UID of the first body.
    bodyB : int
        The UID of the second body.
    linkIndexA : int
        The index of the link on the first body (optional). If not provided,
        checks all links on the body.
    linkIndexB : int
        The index of the link on the second body (optional). If not provided,
        checks all links on the body.
    origin :
        The origin point about which to compute the torque. If ``None`` (the
        default), then the torque is computed about the world frame origin.
    max_contacts :
        The maximum number of contact points expected (optional).
    physicsClientId : int
        The UID of the physics server to use.

    Raises
    ------
    AssertionError
        If ``max_contacts`` is not ``None`` and the number of contact points
        exceeds ``max_contacts``.

    Returns
    -------
    :
        A tuple ``(force, torque)`` representing the contact wrench.
    """
    points = getContactPoints(
        bodyA, bodyB, linkIndexA, linkIndexB, physicsClientId=physicsClientId
    )
    if max_contacts is not None:
        assert len(points) <= max_contacts, f"Found {len(points)} contact points."
    return get_points_contact_wrench(points, origin=origin)
