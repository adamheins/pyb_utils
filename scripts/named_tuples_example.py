#!/usr/bin/env python3
"""Example demonstrating the wrapper functions returning *named* tuples."""
import time

import numpy as np
import pybullet as pyb
import pybullet_data

from pyb_utils.bodies import BulletBody
from pyb_utils.named_tuples import *


STEPS_PER_SEC = 60
TIMESTEP = 1.0 / STEPS_PER_SEC


def main():
    pyb.connect(pyb.GUI)
    pyb.setTimeStep(TIMESTEP)
    pyb.setGravity(0, 0, -9.81)
    pyb.setAdditionalSearchPath(pybullet_data.getDataPath())
    ground_id = pyb.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)

    box = BulletBody.box([0, 0, 0.5], half_extents=[0.5, 0.5, 0.5])

    # dynamics
    info = getDynamicsInfo(box.uid, -1)
    print(f"Box has mass {info.mass}")

    # contact points
    pyb.stepSimulation()
    pts = getContactPoints(box.uid, ground_id)
    print(f"Box has {len(pts)} contact points with the ground. Normal forces:")
    for pt in pts:
        print(pt.normalForce)

    # closest points
    ball = BulletBody.sphere([2, 0, 0.5], radius=0.5)
    pts = getClosestPoints(box.uid, ball.uid, distance=5)
    print(f"Distance between box and ball = {np.min([pt.contactDistance for pt in pts])}")

    # constraint info
    constraint_id = pyb.createConstraint(
        ball.uid,
        -1,
        -1,
        -1,
        pyb.JOINT_FIXED,
        jointAxis=[0, 0, 1],  # doesn't matter
        parentFramePosition=[0, 0, 0],  # location in ball local frame
        childFramePosition=[2, 0, 0.5],  # fix to this position in world frame
    )
    info = getConstraintInfo(constraint_id)
    print(f"Ball fixed at position {info.jointPivotInChild} in the world frame.")



if __name__ == "__main__":
    main()
