#!/usr/bin/env python
"""Example demonstrating collision detection and shortest distance queries."""
import time

import numpy as np
import pybullet as pyb
import pybullet_data

import pyb_utils


TIMESTEP = 1.0 / 60


def load_environment(client_id):
    pyb.setTimeStep(TIMESTEP, physicsClientId=client_id)
    pyb.setAdditionalSearchPath(
        pybullet_data.getDataPath(), physicsClientId=client_id
    )

    # ground plane
    ground_id = pyb.loadURDF(
        "plane.urdf", [0, 0, 0], useFixedBase=True, physicsClientId=client_id
    )

    # KUKA iiwa robot arm
    kuka_id = pyb.loadURDF(
        "kuka_iiwa/model.urdf",
        [0, 0, 0],
        useFixedBase=True,
        physicsClientId=client_id,
    )

    # some cubes for obstacles
    cube1_id = pyb.loadURDF(
        "cube.urdf", [1, 1, 0.5], useFixedBase=True, physicsClientId=client_id
    )
    cube2_id = pyb.loadURDF(
        "cube.urdf", [-1, -1, 0.5], useFixedBase=True, physicsClientId=client_id
    )
    cube3_id = pyb.loadURDF(
        "cube.urdf", [1, -1, 0.5], useFixedBase=True, physicsClientId=client_id
    )

    # store body indices in a dict with more convenient key names
    bodies = {
        "robot": kuka_id,
        "ground": ground_id,
        "cube1": cube1_id,
        "cube2": cube2_id,
        "cube3": cube3_id,
    }
    return bodies


def create_robot_debug_params(robot_id, gui_id):
    """Create debug params to set the robot joint positions from the GUI."""
    params = {}
    for i in range(pyb.getNumJoints(robot_id, physicsClientId=gui_id)):
        joint_name = pyb_utils.getJointInfo(robot_id, i, decode="utf8").jointName
        params[joint_name] = pyb.addUserDebugParameter(
            joint_name,
            rangeMin=-2 * np.pi,
            rangeMax=2 * np.pi,
            startValue=0,
            physicsClientId=gui_id,
        )
    return params


def read_robot_configuration(robot_id, robot_params, gui_id):
    """Read robot configuration from the GUI."""
    n = pyb.getNumJoints(robot_id, physicsClientId=gui_id)
    q = np.zeros(n)

    for i in range(n):
        joint_name = pyb_utils.getJointInfo(robot_id, i, decode="utf8").jointName
        q[i] = pyb.readUserDebugParameter(
            robot_params[joint_name],
            physicsClientId=gui_id,
        )
    return q


def update_robot_configuration(robot_id, q, gui_id):
    """Set the robot configuration."""
    for i in range(pyb.getNumJoints(robot_id, physicsClientId=gui_id)):
        pyb.resetJointState(robot_id, i, q[i], physicsClientId=gui_id)


def main():

    # main simulation server, with a GUI
    gui_id = pyb.connect(pyb.GUI)

    # simulation server only used for collision detection
    col_id = pyb.connect(pyb.DIRECT)

    # add bodies to both of the environments
    bodies = load_environment(gui_id)
    collision_bodies = load_environment(col_id)

    robot_id = bodies["robot"]

    # create user debug parameters
    collision_margin_param = pyb.addUserDebugParameter(
        "collision_margin",
        rangeMin=0,
        rangeMax=0.2,
        startValue=0.01,
        physicsClientId=gui_id,
    )
    robot_params = create_robot_debug_params(robot_id, gui_id)

    # define bodies (and links) to use for shortest distance computations and
    # collision checking
    ground = pyb_utils.NamedCollisionObject("ground")
    cube1 = pyb_utils.NamedCollisionObject("cube1")
    cube2 = pyb_utils.NamedCollisionObject("cube2")
    cube3 = pyb_utils.NamedCollisionObject("cube3")
    link7 = pyb_utils.NamedCollisionObject(
        "robot", "lbr_iiwa_link_7"
    )  # last link

    col_detector = pyb_utils.CollisionDetector(
        col_id,
        collision_bodies,
        [(link7, ground), (link7, cube1), (link7, cube2), (link7, cube3)],
    )

    robot_id = bodies["robot"]

    while True:
        q = read_robot_configuration(robot_id, robot_params, gui_id)

        # compute shortest distances for user-selected configuration
        d = col_detector.compute_distances(q)
        in_col = col_detector.in_collision(
            q, margin=pyb.readUserDebugParameter(collision_margin_param)
        )

        # move to the requested configuration if it is not in collision,
        # otherwise display a warning
        # the key is that we can check collisions using the separate physics
        # client, so we don't have to set the robot to a configuration in the
        # main GUI sim to check if that configuration is in collision
        if not in_col:
            update_robot_configuration(robot_id, q, gui_id=gui_id)
        else:
            pyb.addUserDebugText(
                "Avoiding collision",
                textPosition=[0, 0, 1.5],
                textColorRGB=[1, 0, 0],
                textSize=2,
                lifeTime=0.2,
            )

        print(f"Distance to obstacles = {d}")

        pyb.stepSimulation(physicsClientId=gui_id)
        time.sleep(TIMESTEP)


if __name__ == "__main__":
    main()
