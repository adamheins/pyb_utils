import math

import numpy as np
from spatialmath.base import rotx, roty, rotz
import pyb_utils
import pybullet as pyb


def test_quaternions():
    # 90 deg rotation about z-axis
    angle = np.pi / 2
    q = (0, 0, np.sqrt(2) / 2, np.sqrt(2) / 2)

    C = pyb_utils.quaternion_to_matrix(q)
    assert np.allclose(C, rotz(angle))

    # better to test in rotation matrix form since q2 can be one of two values
    # (negatives of each other) due to double-cover of SO(3)
    q2 = pyb_utils.quaternion_multiply(q, q)
    C2 = pyb_utils.quaternion_to_matrix(q2)
    assert np.allclose(C2, rotz(2 * angle))

    r = pyb_utils.quaternion_rotate(q, [1, 0, 0])
    assert np.allclose(r, [0, 1, 0])


def _test_all_principal_angles(angle):
    assert np.allclose(
        pyb_utils.quatx(angle), pyb_utils.matrix_to_quaternion(rotx(angle))
    )
    assert np.allclose(
        pyb_utils.quaty(angle), pyb_utils.matrix_to_quaternion(roty(angle))
    )
    assert np.allclose(
        pyb_utils.quatz(angle), pyb_utils.matrix_to_quaternion(rotz(angle))
    )


def test_quat_principal_rotations():
    _test_all_principal_angles(np.pi / 4)
    _test_all_principal_angles(2.5 * np.pi)
    _test_all_principal_angles(-np.pi)
    _test_all_principal_angles(-2.5 * np.pi)
