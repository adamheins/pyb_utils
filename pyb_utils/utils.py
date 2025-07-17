import os
from pathlib import Path
import tempfile

import pybullet as pyb


def get_urdf_path():
    """Obtain the path to the extra URDFs packaged with pyb_utils.

    This can be easily integrated into PyBullet using::

        pybullet.setAdditionalSearchPath(pyb_utils.get_urdf_path())

    Returns
    -------
    : str
        The path to the directory containing extra pyb_utils URDFs.
    """
    return (Path(__file__).parent.parent / "urdf").resolve().as_posix()


def load_urdf_from_string(urdf_string, **kwargs):
    """Load a URDF from a string.

    Parameters
    ----------
    urdf_string : str
        The URDF string to load.
    **kwargs : dict
        Additional keyword arguments to pass to the PyBullet loadURDF function.

    Returns
    -------
    : int
        The unique ID of the loaded URDF.
    """
    with tempfile.NamedTemporaryFile(
        suffix=".urdf", mode="w", delete=False
    ) as f:
        # write the URDF
        f.write(urdf_string)
        f.close()

        # load the URDF
        uid = pyb.loadURDF(f.name, **kwargs)

    # remove temp file
    os.remove(f.name)
    return uid
