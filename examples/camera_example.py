#!/usr/bin/env python
"""Example demonstrating camera utilities."""
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pybullet as pyb
import pybullet_data

import pyb_utils


def load_environment(client_id):
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

    bodies = {
        "robot": kuka_id,
        "ground": ground_id,
    }
    return bodies


def main():
    gui_id = pyb.connect(pyb.GUI)
    bodies = load_environment(gui_id)

    camera = pyb_utils.Camera.from_camera_position(
        camera_position=(1, 0, 1),
        target_position=(0, 0, 1),
        near=0.1,
        far=5,
        width=200,
        height=200,
    )

    rgba, depth, seg = camera.get_frame()

    # save the frame
    camera.save_frame("frame.png", rgba=rgba)

    # view point cloud
    points = camera.get_point_cloud(depth=depth)

    # just get points on the robot
    mask = (seg == bodies["robot"])
    points = points[mask, :]

    fig = plt.figure()
    plt.imshow(seg)

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    ax.scatter(points[:, 0], points[:, 1], zs=points[:, 2])
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_aspect("equal")
    plt.show()


if __name__ == "__main__":
    main()
