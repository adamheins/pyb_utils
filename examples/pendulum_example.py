#!/usr/bin/env python3
"""Example of an under-actuated pendulum."""
import time

import numpy as np
import pybullet as pyb
import pybullet_data
import pyb_utils


STEPS_PER_SEC = 60
TIMESTEP = 1.0 / STEPS_PER_SEC


def main():
    pyb.connect(pyb.GUI)
    pyb.setGravity(0, 0, -9.81)
    pyb.setTimeStep(TIMESTEP)

    pyb.setAdditionalSearchPath(pyb_utils.get_urdf_path())
    robot_id = pyb.loadURDF(
        "two_link_planar_pendulum.urdf",
        [0, 0, 1],
        useFixedBase=True,
    )

    # only control the first (top) of the two joints
    robot = pyb_utils.Robot(robot_id, actuated_joint_names=["shaft1_joint"])
    robot.set_joint_friction_forces([0, 0])

    # apply some force
    for _ in range(STEPS_PER_SEC):
        robot.command_effort([10])
        pyb.stepSimulation()
        time.sleep(TIMESTEP)

    # let the system fall under gravity
    for _ in range(5 * STEPS_PER_SEC):
        pyb.stepSimulation()
        time.sleep(TIMESTEP)


if __name__ == "__main__":
    main()
