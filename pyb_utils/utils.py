from pathlib import Path


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
