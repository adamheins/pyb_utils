import pybullet as pyb
import numpy as np

import pyb_utils


def setup_function():
    pyb.connect(pyb.DIRECT)


def teardown_function():
    pyb.disconnect()


def test_body_methods():
    p = [1, 0, 0]
    Q = [1, 0, 0, 0]
    v = [1, 0, 0]
    ω = [0, 1, 0]

    # create the body
    box = pyb_utils.BulletBody.box([0, 0, 0.5], half_extents=[0.5, 0.5, 0.5])

    # set/get pose
    box.set_pose(position=p, orientation=Q)
    assert np.allclose(box.get_pose()[0], p)
    assert np.allclose(box.get_pose()[1], Q)
    assert np.allclose(
        box.get_pose(as_rotation_matrix=True)[1],
        pyb_utils.quaternion_to_matrix(Q),
    )

    # set/get velocity
    box.set_velocity(linear=[1, 0, 0], angular=[0, 1, 0])
    assert np.allclose(box.get_velocity()[0], v)
    assert np.allclose(box.get_velocity()[1], ω)

    box.apply_wrench(force=[1, 0, 0], torque=[0, 0, 5])


def test_body_offset_inertial_frame():
    orn = (0, 0, 1, 0)

    # body base is placed 0.5 units up but inertial frame is then offset 0.5
    # units down: it is at the origin
    box = pyb_utils.BulletBody.box(
        [0, 0, 0.5],
        half_extents=[0.5, 0.5, 0.5],
        baseInertialFramePosition=(0, 0, -0.5),
        baseInertialFrameOrientation=orn,
    )

    r, Q = box.get_pose()
    assert np.allclose(r, np.zeros(3))
    assert np.allclose(Q, orn)


def test_body_types():
    # create each type of body to ensure no errors
    box = pyb_utils.BulletBody.box([0, 0, 0.5], half_extents=[0.5, 0.5, 0.5])
    ball = pyb_utils.BulletBody.sphere(
        [0, 0, 1.5], radius=0.5, color=(0, 1, 0, 1)
    )
    cylinder = pyb_utils.BulletBody.cylinder(
        [0, 1, 1], radius=0.5, height=2, color=(0, 0, 1, 1)
    )
    cap = pyb_utils.BulletBody.capsule(
        [0, 0, 3],
        orientation=(0.707, 0, 0, 0.707),
        radius=0.25,
        height=1,
        color=(1, 0, 1, 1),
    )
