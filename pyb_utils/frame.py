import pybullet as pyb
from pyb_utils.math import quaternion_rotate


def debug_frame_world(size, origin, orientation=(0, 0, 0, 1), line_width=1):
    """Attach a frame to the world for debugging purposes."""
    dx = quaternion_rotate(orientation, [size, 0, 0])
    dy = quaternion_rotate(orientation, [0, size, 0])
    dz = quaternion_rotate(orientation, [0, 0, size])

    pyb.addUserDebugLine(
        origin,
        list(dx + origin),
        lineColorRGB=[1, 0, 0],
        lineWidth=line_width,
    )
    pyb.addUserDebugLine(
        origin,
        list(dy + origin),
        lineColorRGB=[0, 1, 0],
        lineWidth=line_width,
    )
    pyb.addUserDebugLine(
        origin,
        list(dz + origin),
        lineColorRGB=[0, 0, 1],
        lineWidth=line_width,
    )


def debug_frame(size, obj_uid, link_index):
    """Attach a frame to a link for debugging purposes."""
    pyb.addUserDebugLine(
        [0, 0, 0],
        [size, 0, 0],
        lineColorRGB=[1, 0, 0],
        parentObjectUniqueId=obj_uid,
        parentLinkIndex=link_index,
    )
    pyb.addUserDebugLine(
        [0, 0, 0],
        [0, size, 0],
        lineColorRGB=[0, 1, 0],
        parentObjectUniqueId=obj_uid,
        parentLinkIndex=link_index,
    )
    pyb.addUserDebugLine(
        [0, 0, 0],
        [0, 0, size],
        lineColorRGB=[0, 0, 1],
        parentObjectUniqueId=obj_uid,
        parentLinkIndex=link_index,
    )
