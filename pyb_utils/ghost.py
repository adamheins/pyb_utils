"""This module provides utilities for "ghost" (visual-only) objects."""
import numpy as np
import pybullet as pyb

from .math import quaternion_rotate, quaternion_multiply


class GhostObject:
    """A purely visual PyBullet object.

    The GhostObject can be "attached" to another body and positioned relative
    to that body, or it can be positioned at an absolute location in the world
    frame.

    Parameters
    ----------
    visual_uid : int
        The UID of the visual PyBullet object to use.
    position : iterable
        The object's position (optional). Defaults to
        ``(0, 0, 0)``. If ``parent_body_uid`` is also specified, then the
        position is relative to that body, otherwise it is relative to the
        world.
    orientation : iterable
        The object's orientation, as a quaternion :math:`(x, y, z, w)`
        (optional). Defaults to ``(0, 0, 0, 1)``. If ``parent_body_uid`` is
        also specified, then the orientation is relative to that body,
        otherwise it is relative to the world.
    parent_body_uid : int
        The UID of an existing body to "attach" this object to (optional).
    parent_link_index : int
        The index of the link on the parent body to attach this object to
        (optional). Defaults to `-1` (the base link).
    """

    def __init__(
        self,
        visual_uid,
        position=None,
        orientation=None,
        parent_body_uid=None,
        parent_link_index=-1,
        client_id=0,
    ):
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
            physicsClientId=client_id,
        )

    @classmethod
    def sphere(
        cls,
        radius,
        position=None,
        parent_body_uid=None,
        parent_link_index=-1,
        color=(1, 0, 0, 1),
        client_id=0,
    ):
        """Create a spherical ghost object.

        Parameters
        ----------
        radius : float
            The radius of the sphere.
        position : iterable
            The object's position (optional). Defaults to
            ``(0, 0, 0)``. If ``parent_body_uid`` is also specified, then the
            position is relative to that body, otherwise it is relative to the
            world.
        parent_body_uid : int
            The UID of an existing body to "attach" this object to (optional).
        parent_link_index : int
            The index of the link on the parent body to attach this object to
            (optional). Defaults to `-1` (the base link).
        color : iterable
            The ``(r, g, b, α)`` color of the sphere (optional). Defaults to
            red.
        """
        visual_uid = pyb.createVisualShape(
            shapeType=pyb.GEOM_SPHERE,
            radius=radius,
            rgbaColor=color,
        )
        return cls(
            visual_uid,
            position=position,
            parent_body_uid=parent_body_uid,
            parent_link_index=parent_link_index,
            client_id=client_id,
        )

    @classmethod
    def box(
        cls,
        position,
        half_extents,
        parent_body_uid=None,
        parent_link_index=-1,
        color=(1, 0, 0, 1),
        client_id=0,
    ):
        """Create a cuboid ghost.

        Parameters
        ----------
        position : iterable
            The `(x, y, z)` position of the box in the world.
        half_extents : iterable
            The three half lengths of the box.
        parent_body_uid : int
            The UID of an existing body to "attach" this object to (optional).
        parent_link_index : int
            The index of the link on the parent body to attach this object to
            (optional). Defaults to `-1` (the base link).
        color : iterable
            The `(r, g, b, α)` color of the box.
        """

        visual_uid = pyb.createVisualShape(
            shapeType=pyb.GEOM_BOX,
            halfExtents=tuple(half_extents),
            rgbaColor=tuple(color),
        )
        return cls(
            visual_uid,
            position=position,
            parent_body_uid=parent_body_uid,
            parent_link_index=parent_link_index,
            client_id=client_id,
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
        """Update the pose of the object.

        If the object has a parent, then this should be called every time the
        simulation rendering is updated. The object's pose in the world is
        updated to reflect the parent's new pose. If position or orientation is
        supplied, then the pose relative to the parent is updated.

        Otherwise, this function can be used to change the object's absolute
        pose in the world.

        Parameters
        ----------
        position : iterable
            The new position of the object (optional).
        orientation : iterable
            The new orientation of the object, as a quaternion :math:`(x, y, z,
            w)` (optional).
        """
        if position is not None:
            self.position = np.array(position)
        if orientation is not None:
            self.orientation = np.array(orientation)

        world_position, world_orientation = self._compute_world_position()

        pyb.resetBasePositionAndOrientation(
            self.uid, list(world_position), list(world_orientation)
        )


def GhostSphere(*args, **kwargs):
    import logging

    logging.warning(
        "GhostSphere is deprecated. Use GhostObject.sphere instead."
    )
    return GhostObject.sphere(*args, **kwargs)
