import numpy as np
import pybullet as pyb
import pybullet_data
import pyb_utils


def setup_function():
    pyb.connect(pyb.DIRECT)


def teardown_function():
    pyb.disconnect()


def test_collision_detection_basic():
    box1 = pyb_utils.BulletBody.box(
        position=[0, 0, 0.5], half_extents=[0.5, 0.5, 0.5]
    )
    box2 = pyb_utils.BulletBody.box(
        position=[0.5, 0, 0.5], half_extents=[0.5, 0.5, 0.5]
    )

    detector = pyb_utils.CollisionDetector(0, [(box1.uid, box2.uid)])

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


def test_collision_detection_robot():
    pyb.setAdditionalSearchPath(pybullet_data.getDataPath())

    kuka_id = pyb.loadURDF(
        "kuka_iiwa/model.urdf",
        [0, 0, 0],
        useFixedBase=True,
    )
    robot = pyb_utils.Robot(kuka_id)
    box = pyb_utils.BulletBody.box(
        position=[0, 0, 0.5], half_extents=[0.5, 0.5, 0.5]
    )

    # we can treat the robot as one big body
    detector1 = pyb_utils.CollisionDetector(0, [(robot.uid, box.uid)])
    dists1 = detector1.compute_distances()
    assert detector1.in_collision()
    assert len(dists1) == 1

    # or look at each link individually
    robot_col_ids = [(robot.uid, i) for i in range(robot.num_total_joints)]
    col_pairs = [(r, box.uid) for r in robot_col_ids]
    detector2 = pyb_utils.CollisionDetector(0, col_pairs)
    dists2 = detector2.compute_distances()
    assert detector2.in_collision()
    assert len(dists2) == robot.num_total_joints
    assert np.isclose(dists1[0], np.min(dists2))
