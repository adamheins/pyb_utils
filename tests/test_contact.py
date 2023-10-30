import numpy as np
import pybullet as pyb
import pybullet_data
import pyb_utils


GRAVITY = np.array([0, 0, -10])


def setup_function():
    pyb.connect(pyb.DIRECT)
    pyb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pyb.setGravity(*GRAVITY)


def teardown_function():
    pyb.disconnect()


def _sim_env():
    ground_uid = pyb.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)
    box = pyb_utils.BulletBody.box(
        [0, 0, 0.5], half_extents=[0.5, 0.5, 0.5], mass=1.0
    )
    pyb.changeDynamics(ground_uid, -1, lateralFriction=1.0)
    pyb.changeDynamics(box.uid, -1, lateralFriction=0.5)

    # let things settle
    for _ in range(10):
        pyb.stepSimulation()

    return ground_uid, box


def test_contact_wrench_static():
    ground_uid, box = _sim_env()

    # no applied force
    force, torque = pyb_utils.get_total_contact_wrench(box.uid, ground_uid)
    assert np.allclose(force, -GRAVITY, rtol=0, atol=1e-3)
    assert np.allclose(torque, np.zeros(3), rtol=0, atol=1e-3)


def test_contact_wrench_pure_applied_force1():
    ground_uid, box = _sim_env()

    f_app = np.array([1, 0, 0])
    box.apply_wrench(force=f_app, frame=pyb.LINK_FRAME)
    pyb.stepSimulation()
    force, _ = pyb_utils.get_total_contact_wrench(
        box.uid, ground_uid, origin=[0, 0, 0.5]
    )
    assert np.allclose(force, -f_app - GRAVITY, rtol=0, atol=5e-2)


def test_contact_wrench_pure_applied_force2():
    ground_uid, box = _sim_env()

    f_app = np.array([-1, -1, 0])
    box.apply_wrench(force=f_app, frame=pyb.LINK_FRAME)
    pyb.stepSimulation()
    force, torque = pyb_utils.get_total_contact_wrench(
        box.uid, ground_uid, origin=[0, 0, 0.5]
    )
    assert np.allclose(force, -f_app - GRAVITY, rtol=0, atol=5e-2)


def test_contact_wrench_pure_applied_torque():
    ground_uid, box = _sim_env()

    τ_app = np.array([0, 0, 1])
    box.apply_wrench(torque=τ_app, frame=pyb.LINK_FRAME)
    pyb.stepSimulation()
    force, torque = pyb_utils.get_total_contact_wrench(
        box.uid, ground_uid, origin=[0, 0, 0.5]
    )
    assert np.allclose(force, -GRAVITY, rtol=0, atol=1e-2)
    assert np.allclose(torque, -τ_app, rtol=0, atol=1e-2)


def test_contact_wrench_offset_force():
    ground_uid, box = _sim_env()

    f_app = np.array([1, 0, 0])
    p_app = np.array([-0.5, 0.5, 0])
    box.apply_wrench(force=f_app, position=p_app, frame=pyb.LINK_FRAME)
    pyb.stepSimulation()
    force, torque = pyb_utils.get_total_contact_wrench(
        box.uid, ground_uid, origin=[0, 0, 0.5]
    )
    assert np.isclose(force[0], -f_app[0], atol=5e-2)
    assert np.isclose(force[2], -GRAVITY[2], atol=5e-2)
    assert np.isclose(torque[2], -np.cross(p_app, f_app)[2], atol=5e-2)
