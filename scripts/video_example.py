#!/usr/bin/env python
"""Example demonstrating camera utilities."""
import pybullet as pyb
import pybullet_data

from pyb_utils.camera import Camera, VideoRecorder
from pyb_utils.robot import Robot


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

    camera = Camera(
        camera_position=(1, 0, 1),
        target_position=(0, 0, 1),
        near=0.1,
        far=5,
        width=200,
        height=200,
    )

    video = VideoRecorder("test.mp4", camera, fps=10)

    for i in range(100):
        video.capture_frame()


if __name__ == "__main__":
    main()
