"""A simple viewer for URDF files using PyBullet."""

import argparse
import math
import time
import sys

import colorama
import numpy as np
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


def hl(s, color):
    """Colour a string for terminal output."""
    return color + str(s) + colorama.Fore.RESET


def main():
    np.set_printoptions(precision=4, suppress=True)

    parser = argparse.ArgumentParser(
        prog="pybview",
        description="A simple viewer for URDF files using PyBullet.",
    )
    parser.add_argument("urdf_file", type=str, help="The URDF file to load.")
    parser.add_argument(
        "-q",
        "--configuration",
        type=str,
        help="The configuration of the robot, evaluated as a Python expression.",
    )
    parser.add_argument(
        "-g",
        "--gravity",
        type=float,
        default=-9.81,
        help="Acceleration of gravity; defaults to -9.81.",
    )
    parser.add_argument(
        "--no-brakes",
        action="store_true",
        help="Turn off joint brakes (disable joint friction).",
    )
    args = parser.parse_args()

    pyb.connect(pyb.GUI)
    pyb.setGravity(0, 0, args.gravity)

    uid = pyb.loadURDF(args.urdf_file, [0, 0, 0], useFixedBase=True)
    robot = pyb_utils.Robot(uid)

    if args.configuration is not None:
        # joint config can use numpy and math functions
        q = eval(args.configuration, {"np": np, "math": math})
        q = np.array(q, dtype=np.float64)
        nq = q.shape[0]
        if nq != robot.num_moveable_joints:
            print(
                f"Error: expected {robot.num_moveable_joints} config values, got {nq}."
            )
            sys.exit(1)
    else:
        q = np.zeros(robot.num_moveable_joints)
    robot.reset_joint_configuration(q)

    brakes = not args.no_brakes
    if not brakes:
        robot.set_joint_friction_forces(np.zeros(robot.num_moveable_joints))

    joint_info = []
    for i in range(robot.num_total_joints):
        info = pyb_utils.getJointInfo(uid, i, decode="utf-8")
        joint_type = JOINT_TYPE_NAMES[info.jointType]
        if info.jointType == pyb.JOINT_FIXED:
            joint_info.append(f"{info.jointName} ({joint_type})")
        else:
            joint_info.append(
                f"{info.jointName} ({hl(joint_type, color=colorama.Fore.GREEN)})"
            )

    print(f"\nLoaded URDF: {hl(args.urdf_file, color=colorama.Fore.YELLOW)}")
    print(f"q = {q}")
    print(
        f"Joints (total: {robot.num_total_joints}, moveable: {hl(robot.num_moveable_joints, color=colorama.Fore.GREEN)}):"
    )
    print("  " + "\n  ".join(joint_info))
    print("Links:")
    print("  " + "\n  ".join(robot.link_names))
    print(f"Press Ctrl-C to quit.")

    try:
        qd = q.copy()
        while True:
            # hold current position - even with default brakes, robot may drift
            # over time
            if brakes:
                q = robot.get_joint_states()[0]
                robot.command_velocity(qd - q)
            pyb.stepSimulation()
            time.sleep(TIMESTEP)
    except KeyboardInterrupt:
        pass
    finally:
        pyb.disconnect()
