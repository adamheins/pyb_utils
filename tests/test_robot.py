import numpy as np
import pybullet as pyb
import pybullet_data
from robot_descriptions.loaders.pybullet import load_robot_description
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
    assert robot1.num_total_joints == 7
    assert robot1.num_moveable_joints == 7
    assert robot1.num_actuated_joints == 7
    assert robot1.tool_idx == robot1.num_total_joints - 1
    for i in range(robot1.num_total_joints):
        joint_name = f"lbr_iiwa_joint_{i+1}"
        link_name = f"lbr_iiwa_link_{i+1}"

        assert robot1.all_joint_names[i] == joint_name
        assert robot1.moveable_joint_names[i] == joint_name
        assert robot1.actuated_joint_names[i] == joint_name
        assert robot1.link_names[i] == link_name

        assert robot1.get_link_index(link_name) == i
        assert robot1.get_joint_index(joint_name) == i

    # specify the tool joint by name
    robot2 = pyb_utils.Robot(kuka_id, tool_link_name="lbr_iiwa_link_7")
    assert robot2.num_total_joints == 7
    assert robot2.num_moveable_joints == 7
    assert robot2.num_actuated_joints == 7
    assert robot2.tool_idx == robot2.num_total_joints - 1

    # set tool as something other than last joint/link
    robot3 = pyb_utils.Robot(kuka_id, tool_link_name="lbr_iiwa_link_6")
    assert robot3.num_total_joints == 7
    assert robot3.num_moveable_joints == 7
    assert robot3.num_actuated_joints == 7
    assert robot3.tool_idx == robot3.num_total_joints - 2

    # reduced number of actuated joints
    robot4 = pyb_utils.Robot(
        kuka_id,
        actuated_joint_names=[f"lbr_iiwa_joint_{i+1}" for i in range(5)],
    )
    assert robot4.num_total_joints == 7
    assert robot4.num_moveable_joints == 7
    assert robot4.num_actuated_joints == 5


def test_robot_joint_states():
    kuka_id = pyb.loadURDF(
        "kuka_iiwa/model.urdf",
        [0, 0, 0],
        useFixedBase=True,
    )
    robot = pyb_utils.Robot(kuka_id)

    q, v = robot.get_joint_states()
    assert np.allclose(q, np.zeros(robot.num_moveable_joints))
    assert np.allclose(v, np.zeros(robot.num_moveable_joints))

    # Velocity commands are done via motor control and so require stepping the
    # simulation. Setting the joint positions directly overrides the physics
    # and takes effect instantly.
    vd = np.ones(robot.num_actuated_joints)
    robot.command_velocity(vd)
    pyb.stepSimulation()
    _, v = robot.get_joint_states()
    assert np.allclose(v, vd, rtol=0, atol=1e-3)

    qd = np.ones(robot.num_moveable_joints)
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
    r0, Q0 = robot.get_link_frame_pose()
    C0 = pyb_utils.quaternion_to_matrix(Q0)

    # compare frame and CoM positions
    offset = pyb_utils.getLinkState(
        robot.uid, robot.tool_idx
    ).localInertialFramePosition
    r0_com = robot.get_link_com_pose()[0]
    assert np.allclose(r0_com, r0 + C0 @ offset)

    Qz = pyb_utils.quatz(np.pi / 2)

    robot.reset_joint_configuration([np.pi / 2, 0, 0, 0, 0, 0, 0])
    r1, Q1 = robot.get_link_frame_pose()
    C1 = pyb_utils.quaternion_to_matrix(Q1)
    C1_0 = robot.get_link_frame_pose(as_rotation_matrix=True)[1]
    assert np.allclose(r0, r1)
    assert np.allclose(Q1, pyb_utils.quaternion_multiply(Qz, Q0))
    assert np.allclose(C1, C1_0)

    r1_com = robot.get_link_com_pose()[0]
    assert np.allclose(r1_com, r1 + C1 @ offset)

    robot.reset_joint_configuration([0, np.pi / 2, 0, 0, 0, 0, 0])
    r2 = robot.get_link_frame_pose()[0]

    robot.reset_joint_configuration([0, -np.pi / 2, 0, 0, 0, 0, 0])
    r3 = robot.get_link_frame_pose()[0]

    assert np.allclose(r2, [-1, 1, 1] * r3)


def test_robot_jacobian():
    kuka_id = pyb.loadURDF(
        "kuka_iiwa/model.urdf",
        [0, 0, 0],
        useFixedBase=True,
    )
    robot = pyb_utils.Robot(kuka_id)

    # send a velocity command
    vd = np.ones(robot.num_actuated_joints)
    robot.command_velocity(vd)
    pyb.stepSimulation()

    # URDF frame
    J = robot.compute_link_frame_jacobian()
    V = np.concatenate(robot.get_link_frame_velocity())
    v = robot.get_joint_states()[1]
    assert np.allclose(V, J @ v, rtol=0, atol=1e-2)

    # inertial frame
    J = robot.compute_link_com_jacobian()
    V = np.concatenate(robot.get_link_com_velocity())
    v = robot.get_joint_states()[1]
    assert np.allclose(V, J @ v, rtol=0, atol=1e-2)

    # compute at another configuration
    q = np.ones(robot.num_moveable_joints)
    robot.reset_joint_configuration(q)

    robot.command_velocity(vd)
    pyb.stepSimulation()

    # URDF frame
    J = robot.compute_link_frame_jacobian()
    V = np.concatenate(robot.get_link_frame_velocity())
    v = robot.get_joint_states()[1]
    assert np.allclose(V, J @ v, rtol=0, atol=1e-2)

    # inertial frame
    J = robot.compute_link_com_jacobian()
    V = np.concatenate(robot.get_link_com_velocity())
    v = robot.get_joint_states()[1]
    assert np.allclose(V, J @ v, rtol=0, atol=1e-2)


def test_robot_joint_effort():
    # NOTE: no explicit assertions in this test (yet); right now it just
    # ensures that these functions can be called
    kuka_id = pyb.loadURDF(
        "kuka_iiwa/model.urdf",
        [0, 0, 0],
        useFixedBase=True,
    )
    robot = pyb_utils.Robot(kuka_id)
    robot.set_joint_friction_forces(np.zeros(robot.num_moveable_joints))
    robot.command_effort(np.ones(robot.num_actuated_joints))


def test_robot_with_fixed_joints():
    ur10_id = load_robot_description(
        "ur10_description",
        basePosition=[0, 0, 0],
        useFixedBase=True,
    )

    # the UR10 has some extra fixed joints
    robot = pyb_utils.Robot(ur10_id)
    assert robot.num_total_joints == 10
    assert robot.num_moveable_joints == 6
    assert robot.num_actuated_joints == 6
