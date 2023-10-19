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
    robot = pyb_utils.Robot(kuka_id, client_id=client_id)

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
    obstacles = {
        "ground": ground_id,
        "cube1": cube1_id,
        "cube2": cube2_id,
        "cube3": cube3_id,
    }
    return robot, obstacles


def create_robot_params_gui(robot):
    """Create debug params to set the robot joint positions from the GUI."""
    params = {}
    for name in robot.moveable_joint_names:
        params[name] = pyb.addUserDebugParameter(
            name,
            rangeMin=-2 * np.pi,
            rangeMax=2 * np.pi,
            startValue=0,
            physicsClientId=robot.client_id,
        )
    return params


def read_robot_params_gui(robot_params_gui, client_id):
    """Read robot configuration from the GUI."""
    return np.array(
        [
            pyb.readUserDebugParameter(
                param,
                physicsClientId=client_id,
            )
            for param in robot_params_gui.values()
        ]
    )


def main():

    # main simulation server, with a GUI
    gui_id = pyb.connect(pyb.GUI)

    # simulation server only used for collision detection
    col_id = pyb.connect(pyb.DIRECT)

    # add bodies to both of the environments
    robot, _ = load_environment(gui_id)
    col_robot, col_obstacles = load_environment(col_id)

    # create user debug parameters
    collision_margin_param_gui = pyb.addUserDebugParameter(
        "collision_margin",
        rangeMin=0,
        rangeMax=0.2,
        startValue=0.01,
        physicsClientId=gui_id,
    )
    robot_params_gui = create_robot_params_gui(robot)

    # define bodies (and links) to use for shortest distance computations and
    # collision checking
    ground = col_obstacles["ground"]
    cube1 = col_obstacles["cube1"]
    cube2 = col_obstacles["cube2"]
    cube3 = col_obstacles["cube3"]
    link7 = (col_robot.uid, "lbr_iiwa_link_7")

    col_detector = pyb_utils.CollisionDetector(
        col_id,
        [(link7, ground), (link7, cube1), (link7, cube2), (link7, cube3)],
    )

    last_dists = 0

    while True:
        q = read_robot_params_gui(robot_params_gui, client_id=gui_id)

        # move to the requested configuration if it is not in collision,
        # otherwise display a warning
        # the key is that we can check collisions using the separate physics
        # client, so we don't have to set the robot to a configuration in the
        # main GUI sim to check if that configuration is in collision
        col_robot.reset_joint_configuration(q)
        if not col_detector.in_collision(
            margin=pyb.readUserDebugParameter(collision_margin_param_gui),
        ):
            robot.reset_joint_configuration(q)
        else:
            pyb.addUserDebugText(
                "Avoiding collision",
                textPosition=[0, 0, 1.5],
                textColorRGB=[1, 0, 0],
                textSize=2,
                lifeTime=0.2,
            )

        # compute shortest distances for user-selected configuration
        dists = col_detector.compute_distances()
        if not np.allclose(last_dists, dists):
            print(f"Distance to obstacles = {dists}")
            last_dists = dists

        time.sleep(TIMESTEP)


if __name__ == "__main__":
    main()
