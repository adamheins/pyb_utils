"""Wrappers for PyBullet methods that return large tuples.

This submodule returns the same information in a named tuple, so the fields can
be easily identified.
"""
from collections import namedtuple

import pybullet as pyb


# field names are camel case to match PyBullet convention
DynamicsInfo = namedtuple(
    "DynamicsInfo",
    [
        "mass",
        "lateralFriction",
        "localInertiaDiagonal",
        "localInerialPos",
        "localInertialOrn",
        "restitution",
        "rollingFriction",
        "spinningFriction",
        "contactDamping",
        "contactStiffness",
        "bodyType",
        "collisionMargin",
    ],
)

ContactPoint = namedtuple(
    "ContactPoint",
    [
        "contactFlag",
        "bodyUniqueIdA",
        "bodyUniqueIdB",
        "linkIndexA",
        "linkIndexB",
        "positionOnA",
        "positionOnB",
        "contactNormalOnB",
        "contactDistance",
        "normalForce",
        "lateralFriction1",
        "lateralFrictionDir1",
        "lateralFriction2",
        "lateralFrictionDir2",
    ],
)

ConstraintInfo = namedtuple(
    "ConstraintInfo",
    [
        "parentBodyUniqueId",
        "parentJointIndex",
        "childBodyUniqueId",
        "childLinkIndex",
        "constraintType",
        "jointAxis",
        "jointPivotInParent",
        "jointPivotInChild",
        "jointFrameOrientationParent",
        "jointFrameOrientationChild",
        "maxAppliedForce",
        "gearRatio",
        "gearAuxLink",
        "relativePositionTarget",
        "erp",
    ],
)

JointInfo = namedtuple(
    "JointInfo",
    [
        "jointIndex",
        "jointName",
        "jointType",
        "qIndex",
        "uIndex",
        "flags",
        "jointDamping",
        "jointFriction",
        "jointLowerLimit",
        "jointUpperLimit",
        "jointMaxForce",
        "jointMaxVelocity",
        "linkName",
        "jointAxis",
        "parentFramePos",
        "parentFrameOrn",
        "parentIndex",
    ],
)


def getDynamicsInfo(bodyUniqueId, linkIndex, physicsClientId=0):
    """Get information about the dynamic properties of a link.

    Parameters
    ----------
    bodyUniqueId : int
        The UID of the body.
    linkIndex : int
        The index of the link on the body.
    physicsClientId : int
        The UID of the physics server to use.

    Returns
    -------
    DynamicsInfo
    """
    return DynamicsInfo(
        *pyb.getDynamicsInfo(
            bodyUniqueId=bodyUniqueId,
            linkIndex=linkIndex,
            physicsClientId=physicsClientId,
        )
    )


def getContactPoints(
    bodyA=-1, bodyB=-1, linkIndexA=-2, linkIndexB=-2, physicsClientId=0
):
    """Get the contact points between two bodies or specific links.

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
    physicsClientId : int
        The UID of the physics server to use.

    Returns
    -------
    :
        A list of ``ContactPoint``.
    """
    points_raw = pyb.getContactPoints(
        bodyA=bodyA,
        bodyB=bodyB,
        linkIndexA=linkIndexA,
        linkIndexB=linkIndexB,
        physicsClientId=physicsClientId,
    )
    return [ContactPoint(*point) for point in points_raw]


def getClosestPoints(
    bodyA, bodyB, distance, linkIndexA=-2, linkIndexB=-2, physicsClientId=0
):
    """Get the closest points between two bodies or specific links.

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
    physicsClientId : int
        The UID of the physics server to use.

    Returns
    -------
    :
        A list of ``ContactPoint`` representing the closest points. Normal
        force is always zero.
    """
    points_raw = pyb.getClosestPoints(
        bodyA=bodyA,
        bodyB=bodyB,
        distance=distance,
        linkIndexA=linkIndexA,
        linkIndexB=linkIndexB,
        physicsClientId=physicsClientId,
    )
    return [ContactPoint(*point) for point in points_raw]


def getConstraintInfo(constraintUniqueId, physicsClientId=0):
    """Get information about a constraint.

    Parameters
    ----------
    constraintUniqueId : int
        UID of the constraint
    physicsClientId : int
        UID of the physics server to use.

    Returns
    -------
    ConstraintInfo
    """
    return ConstraintInfo(
        *pyb.getConstraintInfo(
            constraintUniqueId=constraintUniqueId,
            physicsClientId=physicsClientId,
        )
    )


def getJointInfo(bodyUniqueId, jointIndex, physicsClientId=0, decode=None):
    """Get information about a joint.

    The one difference from the PyBullet API is the addition of the optional
    ``decode`` argument.

    If ``decode`` is not ``None``, then it is used to decode the strings of bytes
    returned by the PyBullet API for the ``jointName`` and ``linkName`` fields.

    Parameters
    ----------
    bodyUniqueId : int
        UID of the body with the joint.
    jointIndex : int
        Index of the joint on the body.
    physicsClientId : int
        The UID of the physics server to use.
    decode : str
        The encoding to use to convert the bytes fields to strings.

    Returns
    -------
    JointInfo
    """
    info = JointInfo(
        *pyb.getJointInfo(
            bodyUniqueId=bodyUniqueId,
            jointIndex=jointIndex,
            physicsClientId=physicsClientId,
        )
    )
    if decode is not None:
        info = info._replace(
            jointName=info.jointName.decode(decode),
            linkName=info.linkName.decode(decode),
        )
    return info
