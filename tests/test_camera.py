import numpy as np
import pybullet as pyb
import pyb_utils

import matplotlib.pyplot as plt


def setup_function():
    pyb.connect(pyb.DIRECT)


def teardown_function():
    pyb.disconnect()


def test_camera_frame_empty():
    height = 100
    width = 200

    camera = pyb_utils.Camera.from_camera_position(
        camera_position=(1, 0, 1),
        target_position=(0, 0, 1),
        near=0.1,
        far=5,
        width=width,
        height=height,
    )

    rgba, depth, seg = camera.get_frame()

    # nothing in the scene, so the values are all uniform
    assert np.all(rgba == 255 * np.ones((height, width, 4)))
    assert np.allclose(depth, np.ones((height, width)))
    assert np.all(seg == -1 * np.ones((height, width)))


def test_camera_frame_object():
    height = 100
    width = 200

    h2 = height // 2
    w2 = width // 2

    box = pyb_utils.BulletBody.box([0, 0, 0], half_extents=[0.5, 0.5, 0.5], color=[1, 0, 0, 1])

    camera = pyb_utils.Camera.from_camera_position(
        camera_position=(1, 0, 0),
        target_position=(0, 0, 0),
        near=0.1,
        far=5,
        width=width,
        height=height,
    )

    rgba, depth, seg = camera.get_frame()

    # we are looking directly at a red ball
    assert rgba[h2, w2, 0] > 150  # ensure substantial red component
    assert np.all(rgba[h2, w2, 1:] == [0, 0, 255])
    assert seg[h2, w2] == box.uid

    depth_lin = camera.linearize_depth(depth)
    points = camera.get_point_cloud(depth=depth)

    # the middle point should be 0.5m away from the camera
    assert np.isclose(depth_lin[h2, w2], 0.5)
    assert np.allclose(points[h2, w2, :], [0.5, 0, 0])

