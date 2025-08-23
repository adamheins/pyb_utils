"""A simple viewer for URDF files using PyBullet."""
import argparse
import time

import pybullet as pyb
import pyb_utils

TIMESTEP = 1.0 / 240.0

JOINT_TYPE_NAMES = {
    pyb.JOINT_REVOLUTE: "revolute",
    pyb.JOINT_PRISMATIC: "prismatic",
    pyb.JOINT_SPHERICAL: "spherical",
    pyb.JOINT_PLANAR: "planar",
    pyb.JOINT_FIXED: "fixed",
}


def main():
    parser = argparse.ArgumentParser(
        prog="pybview",
        description="A simple viewer for URDF files using PyBullet.",
    )
    parser.add_argument("urdf_file", type=str, help="The URDF file to load.")
    args = parser.parse_args()

    pyb.connect(pyb.GUI)
    pyb.setGravity(0, 0, -9.81)

    uid = pyb.loadURDF(args.urdf_file, [0, 0, 0], useFixedBase=True)
    robot = pyb_utils.Robot(uid)

    joint_info = []
    for i in range(robot.num_total_joints):
        info = pyb_utils.getJointInfo(uid, i, decode="utf-8")
        joint_type = JOINT_TYPE_NAMES[info.jointType]
        joint_info.append(f"{info.jointName} ({joint_type})")

    print(f"\nLoaded URDF: {args.urdf_file}")
    print(f"Total joints: {robot.num_total_joints}")
    print(f"Moveable joints: {robot.num_moveable_joints}")
    print(f"Joints: {', '.join(joint_info)}")
    print(f"Links: {', '.join(robot.link_names)}")
    print(f"Press Ctrl-C to quit.\n")

    try:
        while True:
            pyb.stepSimulation()
            time.sleep(TIMESTEP)
    except KeyboardInterrupt:
        pass
