import numpy as np
import pybullet as pyb
import pybullet_data
from spatialmath.base import rotz
import pyb_utils


def setup_function():
    pyb.connect(pyb.DIRECT)
    pyb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pyb.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)


def teardown_function():
    pyb.disconnect()


def test_robot_setup():
    kuka_id = pyb.loadURDF(
        "kuka_iiwa/model.urdf",
        [0, 0, 0],
        useFixedBase=True,
    )

    robot1 = pyb_utils.Robot(kuka_id)
    assert robot1.num_joints == 7
    assert robot1.tool_idx == robot1.num_joints - 1

    # specify the tool joint by name
    robot2 = pyb_utils.Robot(kuka_id, tool_joint_name="lbr_iiwa_joint_7")
    assert robot2.num_joints == 7
    assert robot2.tool_idx == robot2.num_joints - 1

    # set tool as something other than last joint/link
    robot3 = pyb_utils.Robot(kuka_id, tool_joint_name="lbr_iiwa_joint_6")
    assert robot3.num_joints == 7
    assert robot3.tool_idx == robot3.num_joints - 2


def test_robot_joint_states():
    kuka_id = pyb.loadURDF(
        "kuka_iiwa/model.urdf",
        [0, 0, 0],
        useFixedBase=True,
    )
    robot = pyb_utils.Robot(kuka_id)

    q, v = robot.get_joint_states()
    assert np.allclose(q, np.zeros(robot.num_joints))
    assert np.allclose(v, np.zeros(robot.num_joints))

    # Velocity commands are done via motor control and so require stepping the
    # simulation. Setting the joint positions directly overrides the physics
    # and takes effect instantly.
    vd = np.ones(robot.num_joints)
    robot.command_velocity(vd)
    pyb.stepSimulation()
    _, v = robot.get_joint_states()
    assert np.allclose(v, vd, rtol=0, atol=1e-3)

    qd = np.ones(robot.num_joints)
    robot.reset_joint_configuration(qd)
    q, _ = robot.get_joint_states()
    assert np.allclose(q, qd)


def test_robot_link_pose():
    kuka_id = pyb.loadURDF(
        "kuka_iiwa/model.urdf",
        [0, 0, 0],
        useFixedBase=True,
    )
    robot = pyb_utils.Robot(kuka_id)
    r0, Q0 = robot.get_link_pose()

    Cz = rotz(np.pi / 2)
    Qz = pyb_utils.matrix_to_quaternion(Cz)

    robot.reset_joint_configuration([np.pi / 2, 0, 0, 0, 0, 0, 0])
    r1, Q1 = robot.get_link_pose()
    assert np.allclose(r0, r1)
    assert np.allclose(Q1, pyb_utils.quaternion_multiply(Qz, Q0))

    robot.reset_joint_configuration([0, np.pi / 2, 0, 0, 0, 0, 0])
    r2 = robot.get_link_pose()[0]

    robot.reset_joint_configuration([0, -np.pi / 2, 0, 0, 0, 0, 0])
    r3 = robot.get_link_pose()[0]

    assert np.allclose(r2, [-1, 1, 1] * r3)


def test_robot_jacobian():
    kuka_id = pyb.loadURDF(
        "kuka_iiwa/model.urdf",
        [0, 0, 0],
        useFixedBase=True,
    )
    robot = pyb_utils.Robot(kuka_id)

    # send a velocity command
    vd = np.ones(robot.num_joints)
    robot.command_velocity(vd)
    pyb.stepSimulation()

    # get the offset to the CoM of the link
    # note that Jacobian is computed around the link's parent joint by default,
    # whereas velocity from get_link_velocity is computed around the link's CoM
    state = pyb.getLinkState(robot.uid, robot.tool_idx)
    offset = state[2]

    J = robot.jacobian(offset=offset)
    V = np.concatenate(robot.get_link_velocity())
    v = robot.get_joint_states()[1]
    assert np.allclose(V, J @ v, rtol=0, atol=1e-2)

    # compute at another configuration
    q = np.ones(robot.num_joints)
    robot.reset_joint_configuration(q)

    robot.command_velocity(vd)
    pyb.stepSimulation()

    J = robot.jacobian(offset=offset)
    V = np.concatenate(robot.get_link_velocity())
    v = robot.get_joint_states()[1]
    assert np.allclose(V, J @ v, rtol=0, atol=1e-2)
