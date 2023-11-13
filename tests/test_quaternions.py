import numpy as np
import pyb_utils
import pybullet as pyb


def test_quaternions():
    # 90 deg rotation about z-axis
    angle = np.pi / 2
    q = (0, 0, np.sqrt(2) / 2, np.sqrt(2) / 2)

    C = pyb_utils.quaternion_to_matrix(q)
    assert np.allclose(C, pyb_utils.rotz(angle))

    q2 = pyb_utils.quaternion_multiply(q, q)
    assert np.allclose(q2, pyb_utils.quatz(2 * angle))

    r = pyb_utils.quaternion_rotate(q, [1, 0, 0])
    assert np.allclose(r, [0, 1, 0])


def _test_all_principal_angles(angle):
    C2 = pyb_utils.rot2(angle)

    Cx = pyb_utils.rotx(angle)
    assert np.allclose(
        pyb_utils.quatx(angle),
        pyb_utils.matrix_to_quaternion(Cx),
    )
    assert np.allclose(Cx @ [1, 0, 0], [1, 0, 0])
    assert np.allclose(Cx[1:, 1:], C2)

    Cy = pyb_utils.roty(angle)
    assert np.allclose(
        pyb_utils.quaty(angle),
        pyb_utils.matrix_to_quaternion(Cy),
    )
    assert np.allclose(Cy @ [0, 1, 0], [0, 1, 0])
    assert np.allclose(Cy[[0, 2], :][:, [0, 2]], C2.T)

    Cz = pyb_utils.rotz(angle)
    assert np.allclose(
        pyb_utils.quatz(angle),
        pyb_utils.matrix_to_quaternion(Cz),
    )
    assert np.allclose(Cz @ [0, 0, 1], [0, 0, 1])
    assert np.allclose(Cz[:2, :2], C2)


def test_quat_principal_rotations():
    _test_all_principal_angles(np.pi / 4)
    _test_all_principal_angles(2.5 * np.pi)
    _test_all_principal_angles(-np.pi)
    _test_all_principal_angles(-2.5 * np.pi)
