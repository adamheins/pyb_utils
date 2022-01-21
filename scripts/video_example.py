#!/usr/bin/env python
"""Example demonstrating video recording utilities."""
import time
import pybullet as pyb
import pybullet_data
import numpy as np

from pyb_utils.camera import Camera, VideoRecorder
from pyb_utils.robots import Robot


DURATION = 5
DT = 0.01
FPS = 10

SECONDS_PER_FRAME = 1.0 / FPS


def load_environment(client_id):
    pyb.setAdditionalSearchPath(
        pybullet_data.getDataPath(), physicsClientId=client_id
    )
    pyb.setTimeStep(DT)

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

    camera = Camera(
        camera_position=(1, 0, 1),
        target_position=(0, 0, 1),
        near=0.1,
        far=5,
        width=200,
        height=200,
    )
    video = VideoRecorder("example.mp4", camera, fps=FPS)

    qd = np.pi * (np.random.random(robot.num_joints) - 0.5)
    K = np.eye(robot.num_joints)

    t = 0
    t_frame = 0
    video.capture_frame()  # capture the initial frame
    while t <= DURATION:
        # basic joint space velocity control
        q, _ = robot.get_joint_states()
        u = K @ (qd - q)
        robot.command_velocity(u)

        t += DT
        t_frame += DT

        # capture a new frame every SECONDS_PER_FRAME seconds
        if t_frame >= SECONDS_PER_FRAME:
            video.capture_frame()
            t_frame = 0

        time.sleep(DT)
        pyb.stepSimulation()


if __name__ == "__main__":
    main()
