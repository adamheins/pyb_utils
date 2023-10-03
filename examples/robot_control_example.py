#!/usr/bin/env python3
"""Example of basic task-space control of a robot arm."""
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
    pyb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pyb.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)

    kuka_id = pyb.loadURDF(
        "kuka_iiwa/model.urdf",
        [0, 0, 0],
        useFixedBase=True,
    )
    robot = pyb_utils.Robot(kuka_id)
    r = robot.get_link_frame_pose()[0]

    # desired end effector positions
    goals = np.array(
        [[0.75, 0, 0.5], [0, 0.75, 0.5], [-0.75, 0, 0.5], [0, -0.75, 0.5]]
    )

    for goal in goals:
        pyb_utils.debug_frame_world(size=0.1, origin=goal)

    for goal in goals:
        print(f"Goal = {goal}")
        while np.linalg.norm(goal - r) > 0.01:
            r = robot.get_link_frame_pose()[0]
            J = robot.compute_link_frame_jacobian()[:3, :]

            # simple inverse kinematics controller
            u = np.linalg.lstsq(J, goal - r, rcond=None)[0]

            robot.command_velocity(u)
            pyb.stepSimulation()
            time.sleep(TIMESTEP)
    print("Done")


main()
