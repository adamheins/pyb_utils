import numpy as np
import pybullet as pyb

from pyb_utils.math import quaternion_rotate, quaternion_multiply


class GhostObject:
    """A purely visual PyBullet object.

    The GhostObject can be "attached" to another body and positioned relative
    to that body, or it can be positioned at an absolute location in the world
    frame.
    """

    def __init__(
        self,
        visual_uid,
        position=None,
        orientation=None,
        parent_body_uid=None,
        parent_link_index=-1,
    ):
        """Initialize the GhostObject.

        Parameters:
            visual_uid: The unique index of the visual PyBullet object to use.
            position: Optionally provide the object's position. Defaults to
                (0, 0, 0). If `parent_body_uid` is also specified, then the
                position is relative to that body, otherwise it is absolute.
            orientation: Optionally provide the object's orientation, as a
                quaternion using xyzw ordering. Defaults to (0, 0, 0, 1). If
                `parent_body_uid` is also specified, then the orientation is
                relative to that body, otherwise it is absolute.
            parent_body_uid: Optionally provide the unique index of an existing
                body to "attach" this object to.
            parent_link_index: The index of the link on the parent body to
                attach this object to. Defaults to -1 (the base link).
        """
        if position is None:
            position = (0, 0, 0)
        if orientation is None:
            orientation = (0, 0, 0, 1)

        self.position = np.array(position)
        self.orientation = np.array(orientation)

        self.parent_body_uid = parent_body_uid
        self.parent_link_index = parent_link_index

        world_position, world_orientation = self._compute_world_position()

        self.uid = pyb.createMultiBody(
            baseMass=0,  # non-dynamic body has mass = 0
            baseVisualShapeIndex=visual_uid,
            basePosition=list(world_position),
            baseOrientation=list(world_orientation),
        )

    def _compute_world_position(self):
        # If the object is attached to a parent, then its position and
        # orientation are relative to the parent
        if self.parent_body_uid is not None:
            state = pyb.getLinkState(
                self.parent_body_uid,
                self.parent_link_index,
                computeForwardKinematics=True,
            )
            parent_position, parent_orientation = state[4], state[5]

            # compute world pose given parent pose and relative pose
            world_position = parent_position + quaternion_rotate(
                parent_orientation, self.position
            )
            world_orientation = quaternion_multiply(
                parent_orientation, self.orientation
            )
            return world_position, world_orientation

        # Otherwise, the position and orientation are already in the world
        # frame
        return self.position, self.orientation

    def update(self, position=None, orientation=None):
        """Update the pose of the object, optionally updating position and/or
        orientation.

        If the object has a parent, then this should be called every time the
        simulation rendering is updated. The object's pose in the world is
        updated to reflect the parent's new pose. If position or orientation is
        supplied, then the pose relative to the parent is updated.

        Otherwise, this function can be used to change the object's absolute
        pose in the world.
        """
        if position is not None:
            self.position = np.array(position)
        if orientation is not None:
            orientation = np.array(orientation)

        world_position, world_orientation = self._compute_world_position()

        pyb.resetBasePositionAndOrientation(
            self.uid, list(world_position), list(world_orientation)
        )


class GhostSphere(GhostObject):
    """Spherical ghost object."""

    def __init__(
        self,
        radius,
        position=None,
        parent_body_uid=None,
        parent_link_index=-1,
        color=(1, 0, 0, 0),
    ):
        visual_uid = pyb.createVisualShape(
            shapeType=pyb.GEOM_SPHERE,
            radius=radius,
            rgbaColor=color,
        )
        super().__init__(
            visual_uid,
            position=position,
            parent_body_uid=parent_body_uid,
            parent_link_index=parent_link_index,
        )
