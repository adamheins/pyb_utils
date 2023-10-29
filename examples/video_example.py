#!/usr/bin/env python
"""Example demonstrating video recording utilities."""
import time

import numpy as np
import pybullet as pyb
import pybullet_data

import pyb_utils


DURATION = 5
TIMESTEP = 0.01
FPS = 10

SECONDS_PER_FRAME = 1.0 / FPS


def load_environment(client_id):
    pyb.setAdditionalSearchPath(
        pybullet_data.getDataPath(), physicsClientId=client_id
    )
    pyb.setTimeStep(TIMESTEP)

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

    return pyb_utils.Robot(kuka_id)


def main():
    gui_id = pyb.connect(pyb.GUI)
    robot = load_environment(gui_id)

    camera = pyb_utils.Camera.from_camera_position(
        camera_position=(1, 0, 1),
        target_position=(0, 0, 1),
        near=0.1,
        far=5,
        width=200,
        height=200,
    )
    video = pyb_utils.VideoRecorder("example.mp4", camera, fps=FPS)

    # random target configuration
    qd = np.pi * (np.random.random(robot.num_moveable_joints) - 0.5)
    K = np.eye(robot.num_moveable_joints)

    t = 0
    t_frame = 0
    video.capture_frame()  # capture the initial frame
    while t <= DURATION:
        # basic joint space velocity control
        q, _ = robot.get_joint_states()
        u = K @ (qd - q)
        robot.command_velocity(u)

        t += TIMESTEP
        t_frame += TIMESTEP

        # capture a new frame every SECONDS_PER_FRAME seconds
        if t_frame >= SECONDS_PER_FRAME:
            video.capture_frame()
            t_frame = 0

        time.sleep(TIMESTEP)
        pyb.stepSimulation()


if __name__ == "__main__":
    main()
