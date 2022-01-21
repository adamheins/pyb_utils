#!/usr/bin/env python
"""Example demonstrating the use of ghost objects.

These objects are purely visual objects that can be added to PyBullet (i.e.,
they don't influence the simulated physics at all). These could be used to
visualize volumes like collision spheres programmatically, for example.
"""

import time

import numpy as np
import pybullet as pyb
import pybullet_data

from pyb_utils.ghost import GhostSphere
from pyb_utils.robot import Robot


DT = 1.0 / 240


def load_environment(client_id):
    pyb.setAdditionalSearchPath(
        pybullet_data.getDataPath(), physicsClientId=client_id
    )

    # ground plane
    pyb.loadURDF(
        "plane.urdf", [0, 0, 0], useFixedBase=True, physicsClientId=client_id
    )

    # KUKA iiwa robot arm
    kuka_id = pyb.loadURDF(
        "kuka_iiwa/model.urdf",
        [0, 0, 0],
        useFixedBase=True,
        physicsClientId=client_id,
    )

    return Robot(kuka_id)


def main():
    gui_id = pyb.connect(pyb.GUI)
    robot = load_environment(gui_id)

    # choose a random desired configuration to control the robot toward
    qd = np.pi * (np.random.random(robot.num_joints) - 0.5)
    K = np.eye(robot.num_joints)

    # fixed sphere above the robot
    ghost_fixed = GhostSphere(
        radius=0.1, color=(0, 1, 0, 0.5), position=(0, 0, 2)
    )

    # sphere attached to the robot's end effector
    ghost_relative = GhostSphere(
        radius=0.1,
        color=(1, 0, 0, 0.5),
        parent_body_uid=robot.uid,
        parent_link_index=robot.num_joints - 1,
    )

    while True:
        # basic joint space velocity control
        q, _ = robot.get_joint_states()
        u = K @ (qd - q)
        robot.command_velocity(u)

        # we only need to update the relative sphere, because its pose needs to
        # be recomputed whenever its parent (the robot) moves
        ghost_relative.update()

        time.sleep(DT)
        pyb.stepSimulation()


if __name__ == "__main__":
    main()
