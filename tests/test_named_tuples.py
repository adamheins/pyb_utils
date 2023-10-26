import numpy as np
import pybullet as pyb
import pybullet_data
import pyb_utils


def setup_function():
    pyb.connect(pyb.DIRECT)


def teardown_function():
    pyb.disconnect()


def uniform_cuboid_inertia(mass, half_extents):
    """Inertia matrix of a cuboid with given half lengths."""
    lx, ly, lz = 2 * np.array(half_extents)
    xx = ly**2 + lz**2
    yy = lx**2 + lz**2
    zz = lx**2 + ly**2
    return mass * np.diag([xx, yy, zz]) / 12.0


def test_dynamics_info():
    mass = 1.0
    half_extents = 0.5 * np.ones(3)
    box = pyb_utils.BulletBody.box(
        position=[0, 0, 0.5], half_extents=half_extents, mass=mass
    )

    info = pyb_utils.getDynamicsInfo(box.uid, -1)
    assert np.isclose(info.mass, 1.0)
    assert np.allclose(
        info.localInertiaDiagonal,
        np.diag(uniform_cuboid_inertia(mass, half_extents)),
    )


def test_contact_points():
    pyb.setGravity(0, 0, -10)
    pyb.setAdditionalSearchPath(pybullet_data.getDataPath())
    ground_id = pyb.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)

    ball = pyb_utils.BulletBody.sphere(
        position=[0, 0, 0.5], radius=0.5, mass=1.0
    )

    # step a few times to let things settle
    pyb.stepSimulation()
    pyb.stepSimulation()
    pyb.stepSimulation()

    points = pyb_utils.getContactPoints(ball.uid, ground_id)
    assert len(points) == 1
    assert abs(points[0].contactDistance) <= 1e-4
    assert np.isclose(points[0].normalForce, 10)


def test_closest_points():
    pyb.setGravity(0, 0, 10)
    pyb.setAdditionalSearchPath(pybullet_data.getDataPath())
    ground_id = pyb.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)

    ball = pyb_utils.BulletBody.sphere(
        position=[0, 0, 0.5], radius=0.5, mass=1.0
    )

    pyb.stepSimulation()

    points = pyb_utils.getClosestPoints(ball.uid, ground_id, distance=1)
    assert len(points) == 1
    assert abs(points[0].contactDistance) <= 1e-3
    assert np.isclose(points[0].normalForce, 0)


def test_constraint_info():
    ball = pyb_utils.BulletBody.sphere(
        position=[0, 0, 0.5], radius=0.5, mass=1.0
    )

    constraint_id = pyb.createConstraint(
        ball.uid,
        -1,
        -1,
        -1,
        pyb.JOINT_FIXED,
        jointAxis=[0, 0, 1],  # doesn't matter
        parentFramePosition=[0, 0, 0],  # location in ball local frame
        childFramePosition=[2, 0, 0.5],  # fix to this position in world frame
    )
    info = pyb_utils.getConstraintInfo(constraint_id)
    assert np.allclose(info.jointPivotInParent, np.zeros(3))
    assert np.allclose(info.jointPivotInChild, [2, 0, 0.5])


def test_joint_info():
    pyb.setAdditionalSearchPath(pybullet_data.getDataPath())
    robot = pyb_utils.Robot(
        pyb.loadURDF(
            "kuka_iiwa/model.urdf",
            [0, 0, 0],
            useFixedBase=True,
        )
    )

    # names are as expected when decoded
    info = pyb_utils.getJointInfo(robot.uid, 2, decode="utf8")
    assert info.jointIndex == 2
    assert info.jointName == "lbr_iiwa_joint_3"
    assert info.linkName == "lbr_iiwa_link_3"

    info = pyb_utils.getJointInfo(robot.uid, 2, decode=None)
    assert info.jointName != "lbr_iiwa_joint_3"
    assert info.linkName != "lbr_iiwa_link_3"


def test_joint_states():
    pyb.setAdditionalSearchPath(pybullet_data.getDataPath())
    robot = pyb_utils.Robot(
        pyb.loadURDF(
            "kuka_iiwa/model.urdf",
            [0, 0, 0],
            useFixedBase=True,
        )
    )

    state = pyb_utils.getJointState(robot.uid, 2)
    assert np.isclose(state.jointPosition, 0)

    # no joint reaction forces reported with F/T sensor
    assert np.allclose(state.jointReactionForces, np.zeros(6))

    # now get the reaction forces with the F/T sensor
    pyb.enableJointForceTorqueSensor(robot.uid, 2, enableSensor=1)
    robot.command_velocity(np.ones(robot.num_actuated_joints))
    pyb.stepSimulation()
    state = pyb_utils.getJointState(robot.uid, 2)
    assert np.linalg.norm(state.jointReactionForces) > 0

    states = pyb_utils.getJointStates(robot.uid, np.arange(robot.num_total_joints))
    assert len(states) == robot.num_total_joints


def test_link_states():
    pyb.setAdditionalSearchPath(pybullet_data.getDataPath())
    robot = pyb_utils.Robot(
        pyb.loadURDF(
            "kuka_iiwa/model.urdf",
            [0, 0, 0],
            useFixedBase=True,
        )
    )

    # we get None for velocities when computeLinkVelocity=False
    state = pyb_utils.getLinkState(robot.uid, 2)
    assert state.worldLinkLinearVelocity is None
    assert state.worldLinkAngularVelocity is None

    state = pyb_utils.getLinkState(robot.uid, 2, computeLinkVelocity=True)
    assert np.allclose(state.worldLinkLinearVelocity, np.zeros(3))
    assert np.allclose(state.worldLinkAngularVelocity, np.zeros(3))

    # forward kinematics only updated after stepping the sim when
    # computeForwardKinematics=True
    for i in range(10):
        robot.command_velocity(np.ones(7))
        pyb.stepSimulation()

    state1 = pyb_utils.getLinkState(robot.uid, 2)
    state2 = pyb_utils.getLinkState(robot.uid, 2, computeForwardKinematics=True)
    assert not np.allclose(state1.linkWorldPosition, state2.linkWorldPosition)
    assert not np.allclose(state1.linkWorldOrientation, state2.linkWorldOrientation)
    assert not np.allclose(state1.worldLinkFramePosition, state2.worldLinkFramePosition)
    assert not np.allclose(state1.worldLinkFrameOrientation, state2.worldLinkFrameOrientation)

    states = pyb_utils.getLinkStates(robot.uid, np.arange(robot.num_total_joints))
    assert len(states) == robot.num_total_joints
