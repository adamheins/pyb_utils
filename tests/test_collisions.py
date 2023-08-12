import numpy as np
import pybullet as pyb
import pybullet_data
import pyb_utils


def setup_function():
    pyb.connect(pyb.DIRECT)


def teardown_function():
    pyb.disconnect()


def test_collision_detection():
    box1 = pyb_utils.BulletBody.box(
        position=[0, 0, 0.5], half_extents=[0.5, 0.5, 0.5]
    )
    box2 = pyb_utils.BulletBody.box(
        position=[0.5, 0, 0.5], half_extents=[0.5, 0.5, 0.5]
    )

    uids = {
        "box1": box1.uid,
        "box2": box2.uid,
    }

    box1_col = pyb_utils.NamedCollisionObject("box1")
    box2_col = pyb_utils.NamedCollisionObject("box2")
    detector = pyb_utils.CollisionDetector(0, uids, [(box1_col, box2_col)])

    # boxes are initially in collision
    dists = detector.compute_distances()
    assert len(dists) == 1
    assert np.isclose(dists[0], -0.5)
    assert detector.in_collision()

    # move boxes out of collision
    box2.set_pose(position=[1.5, 0, 0])
    dists = detector.compute_distances()
    assert len(dists) == 1
    assert np.isclose(dists[0], 0.5)
    assert not detector.in_collision()
