"""This modules provides visible oriented frames for debugging the simulation."""

from typing import Iterable, Tuple

import pybullet as pyb

from .math import quaternion_rotate


def debug_frame_world(
    size: float,
    origin: Iterable[float],
    orientation: Iterable[float] = (0, 0, 0, 1),
    line_width: float = 1,
) -> Tuple[int]:
    """Attach a frame to the world for debugging purposes.

    Parameters
    ----------
    size : float
        The length of each arm of the frame.
    origin : iterable
        The origin of the frame.
    orientation : iterable
        A quaternion :math:`(x, y, z, w)` represented the frame's orientation.
    line_width : float
        Width of the lines that make up the frame.
    Returns
    -------
    : tuple[int]
        pybullet ids of the lines
    """
    dx = quaternion_rotate(orientation, [size, 0, 0])
    dy = quaternion_rotate(orientation, [0, size, 0])
    dz = quaternion_rotate(orientation, [0, 0, size])

    l1 = pyb.addUserDebugLine(
        origin,
        list(dx + origin),
        lineColorRGB=[1, 0, 0],
        lineWidth=line_width,
    )
    l2 = pyb.addUserDebugLine(
        origin,
        list(dy + origin),
        lineColorRGB=[0, 1, 0],
        lineWidth=line_width,
    )
    l3 = pyb.addUserDebugLine(
        origin,
        list(dz + origin),
        lineColorRGB=[0, 0, 1],
        lineWidth=line_width,
    )
    return l1, l2, l3


def debug_frame(
    size: float, obj_uid: int, link_index: int, line_width: int = 1
) -> Tuple[int]:
    """Attach a frame to a link for debugging purposes.

    Parameters
    ----------
    size : float
        The length of each arm of the frame.
    obj_uid : int
        UID of the body to attach the frame to.
    link_index : int
        Index of the link on the body to attach the frame to. Use `-1` for the
        base link.
    Returns
    -------
    : tuple[int]
        pybullet ids of the lines
    """
    l1 = pyb.addUserDebugLine(
        [0, 0, 0],
        [size, 0, 0],
        lineColorRGB=[1, 0, 0],
        parentObjectUniqueId=obj_uid,
        parentLinkIndex=link_index,
        lineWidth=line_width,
    )
    l2 = pyb.addUserDebugLine(
        [0, 0, 0],
        [0, size, 0],
        lineColorRGB=[0, 1, 0],
        parentObjectUniqueId=obj_uid,
        parentLinkIndex=link_index,
        lineWidth=line_width,
    )
    l3 = pyb.addUserDebugLine(
        [0, 0, 0],
        [0, 0, size],
        lineColorRGB=[0, 0, 1],
        parentObjectUniqueId=obj_uid,
        parentLinkIndex=link_index,
        lineWidth=line_width,
    )
    return l1, l2, l3
