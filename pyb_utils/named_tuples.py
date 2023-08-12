"""Wrappers for PyBullet methods that return large tuples.

This submodule returns the same info in a named tuple, so the fields can be
easily identified.
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
    return ConstraintInfo(
        *pyb.getConstraintInfo(
            constraintUniqueId=constraintUniqueId,
            physicsClientId=physicsClientId,
        )
    )


def getJointInfo(bodyUniqueId, jointIndex, physicsClientId=0, decode=None):
    """The one difference from the PyBullet API is the addition of the optional
    `decode` argument.

    If `decode` is not None, then it is used to decode the strings of bytes
    returned by the PyBullet API for the `jointName` and `linkName` fields.
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
