#!/usr/bin/env python3
"""Example demonstrating the use of the BulletBody convenience class to create
rigid bodies."""
import time

import pybullet as pyb
import pybullet_data

from pyb_utils.bodies import BulletBody


STEPS_PER_SEC = 60
TIMESTEP = 1.0 / STEPS_PER_SEC


def main():
    pyb.connect(pyb.GUI)
    pyb.setTimeStep(TIMESTEP)
    pyb.setGravity(0, 0, -9.81)
    pyb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pyb.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)

    # create some objects
    box = BulletBody.box([0, 0, 0.5], half_extents=[0.5, 0.5, 0.5])
    ball = BulletBody.sphere([0, 0, 1.5], radius=0.5, color=(0, 1, 0, 1))
    cylinder = BulletBody.cylinder(
        [0, 1, 1], radius=0.5, height=2, color=(0, 0, 1, 1)
    )
    cap = BulletBody.capsule(
        [0, 0, 3],
        orientation=(0.707, 0, 0, 0.707),
        radius=0.25,
        height=1,
        color=(1, 0, 1, 1),
    )

    # set velocity
    for _ in range(2 * STEPS_PER_SEC):
        box.set_velocity(linear=[1, 0, 0])
        pyb.stepSimulation()
        time.sleep(TIMESTEP)

    # start applying some torque as well
    for _ in range(2 * STEPS_PER_SEC):
        box.apply_wrench(torque=[0, 0, 5])
        pyb.stepSimulation()
        time.sleep(TIMESTEP)

    # stop spinning
    box.set_velocity(angular=[0, 0, 0])
    for _ in range(2 * STEPS_PER_SEC):
        pyb.stepSimulation()
        time.sleep(TIMESTEP)


if __name__ == "__main__":
    main()
