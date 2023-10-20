import numpy as np
from spatialmath.base import rotz
import pyb_utils


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
