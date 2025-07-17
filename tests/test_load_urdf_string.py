import os
import pybullet as pyb
import pyb_utils


def test_load_urdf_string():
    pyb.connect(pyb.DIRECT)

    # load the URDF string directly
    path = os.path.join(pyb_utils.get_urdf_path(), "two_link_planar_pendulum.urdf")
    with open(path, "r") as f:
        urdf_string = f.read()

    uid = pyb_utils.load_urdf_from_string(urdf_string)
    assert uid >= 0, "Failed to load URDF from string"

    pyb.disconnect()
