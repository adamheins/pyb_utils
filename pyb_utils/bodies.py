"""This module provides a utility class for creating rigid bodies in PyBullet."""
import pybullet as pyb
import numpy as np

from .math import quaternion_to_matrix


class BulletBody:
    """Generic rigid body in PyBullet.

    Parameters
    ----------
    position : iterable
        The :math:`(x, y, z)` position of the body in the world.
    collision_uid : int
        ID of the collision shape to use.
    visual_uid : int
        ID of the visual shape to use.
    mass : float
        mass of the body; defaults to `1`.
    orientation : iterable
        A quaternion :math:`(x, y, z, w)` representing the orientation of the body; if
        not provided, orientation is aligned with the world frame axes.
    client_id : int
        physics client ID; only required if connected to multiple servers.
    """

    def __init__(
        self,
        position,
        collision_uid,
        visual_uid,
        mass=1,
        orientation=None,
        client_id=0,
        **kwargs
    ):
        if orientation is None:
            orientation = (0, 0, 0, 1)

        self.client_id = client_id
        self.uid = pyb.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_uid,
            baseVisualShapeIndex=visual_uid,
            basePosition=tuple(position),
            baseOrientation=tuple(orientation),
            physicsClientId=client_id,
            **kwargs,
        )

    @classmethod
    def box(
        cls, position, half_extents, color=(1, 0, 0, 1), client_id=0, **kwargs
    ):
        """Create a cuboid body.

        Parameters
        ----------
        position : iterable
            The `(x, y, z)` position of the box in the world.
        half_extents : iterable
            The three half lengths of the box.
        color : iterable
            The `(r, g, b, α)` color of the box.
        client_id : int
            Physics client ID; only required if connected to multiple servers.
        """
        collision_uid = pyb.createCollisionShape(
            shapeType=pyb.GEOM_BOX,
            halfExtents=tuple(half_extents),
            physicsClientId=client_id,
        )
        visual_uid = pyb.createVisualShape(
            shapeType=pyb.GEOM_BOX,
            halfExtents=tuple(half_extents),
            rgbaColor=tuple(color),
            physicsClientId=client_id,
        )
        return cls(
            position, collision_uid, visual_uid, client_id=client_id, **kwargs
        )

    @classmethod
    def sphere(
        cls, position, radius, color=(1, 0, 0, 1), client_id=0, **kwargs
    ):
        """Create a spherical body.

        Parameters
        ----------
        position : iterable
            The `(x, y, z)` position of the sphere in the world.
        radius : float
            The radius of the sphere.
        color : iterable
            The `(r, g, b, α)` color of the sphere.
        client_id : int
            Physics client ID; only required if connected to multiple servers.
        """
        collision_uid = pyb.createCollisionShape(
            shapeType=pyb.GEOM_SPHERE,
            radius=radius,
            physicsClientId=client_id,
        )
        visual_uid = pyb.createVisualShape(
            shapeType=pyb.GEOM_SPHERE,
            radius=radius,
            rgbaColor=color,
            physicsClientId=client_id,
        )
        return cls(
            position, collision_uid, visual_uid, client_id=client_id, **kwargs
        )

    @classmethod
    def cylinder(
        cls, position, radius, height, color=(1, 0, 0, 1), client_id=0, **kwargs
    ):
        """Create a cylindrical body.

        The cylinder is oriented along its z-axis, which corresponds to the
        height.

        Parameters
        ----------
        position : iterable
            The `(x, y, z)` position of the cylinder in the world.
        radius : float
            The radius of the cylinder.
        height: float
            The height of the cylinder.
        color : iterable
            The `(r, g, b, α)` color of the cylinder.
        client_id : int
            Physics client ID; only required if connected to multiple servers.
        """
        collision_uid = pyb.createCollisionShape(
            shapeType=pyb.GEOM_CYLINDER,
            radius=radius,
            height=height,
            physicsClientId=client_id,
        )
        visual_uid = pyb.createVisualShape(
            shapeType=pyb.GEOM_CYLINDER,
            radius=radius,
            length=height,
            rgbaColor=color,
            physicsClientId=client_id,
        )
        return cls(
            position, collision_uid, visual_uid, client_id=client_id, **kwargs
        )

    @classmethod
    def capsule(
        cls, position, radius, height, color=(1, 0, 0, 1), client_id=0, **kwargs
    ):
        """Create a capsular body. A capsule is a cylinder with half-spheres on
        the ends.

        The capsule is oriented along its z-axis, which corresponds to the
        height.

        Parameters
        ----------
        position : iterable
            The `(x, y, z)` position of the capsule in the world.
        radius : float
            The radius of the capsule.
        height: float
            The height of the capsule.
        color : iterable
            The `(r, g, b, α)` color of the capsule.
        client_id : int
            Physics client ID; only required if connected to multiple servers.
        """
        collision_uid = pyb.createCollisionShape(
            shapeType=pyb.GEOM_CAPSULE,
            radius=radius,
            height=height,
            physicsClientId=client_id,
        )
        visual_uid = pyb.createVisualShape(
            shapeType=pyb.GEOM_CAPSULE,
            radius=radius,
            length=height,
            rgbaColor=color,
            physicsClientId=client_id,
        )
        return cls(
            position, collision_uid, visual_uid, client_id=client_id, **kwargs
        )

    def get_pose(self, as_rotation_matrix=False):
        """Get the position and orientation of the body's inertial frame.

        Parameters
        ----------
        as_rotation_matrix : bool
            Set to ``True`` to return the orientation as a rotation matrix,
            ``False`` to return a quaternion.

        Returns
        -------
        :
            A tuple containing the position and orientation of the link's
            center of mass in the world frame. If ``as_rotation_matrix=True``,
            then the orientation is represented as a :math:`3\\times3` rotation
            matrix. If ``False``, then it is represented as a quaternion in
            xyzw order.
        """
        pos, orn = pyb.getBasePositionAndOrientation(
            self.uid, physicsClientId=self.client_id
        )
        pos = np.array(pos)
        orn = np.array(orn)
        orn /= np.linalg.norm(orn)
        if as_rotation_matrix:
            orn = quaternion_to_matrix(orn)
        return pos, orn

    def get_velocity(self):
        """Get the velocity of the body's inertial frame.

        Returns
        -------
        :
            A tuple ``(linear, angular)`` containing the linear and angular
            velocity, each an array of length 3.
        """
        linear, angular = pyb.getBaseVelocity(
            self.uid, physicsClientId=self.client_id
        )
        return np.array(linear), np.array(angular)

    def set_pose(self, position=None, orientation=None):
        """Set the position and orientation of the body's inertial frame.

        Parameters
        ----------
        position : iterable
            The :math:`(x, y, z)` position of the body in the world.
        orientation : iterable
            A quaternion :math:`(x, y, z, w)` representing the orientation of
            the body.
        """
        current_pos, current_orn = self.get_pose()
        if position is None:
            position = current_pos
        if orientation is None:
            orientation = current_orn
        pyb.resetBasePositionAndOrientation(
            self.uid,
            list(position),
            list(orientation),
            physicsClientId=self.client_id,
        )

    def set_velocity(self, linear=None, angular=None):
        """Set the velocity of the body's inertial frame.

        Parameters
        ----------
        linear : iterable
            Linear velocity of the body. If not provided, linear velocity is
            unchanged.
        angular : iterable
            Angular velocity of the body. If not provided, angular velocity is
            unchanged.
        """
        current_linear, current_angular = self.get_velocity()
        if linear is None:
            linear = current_linear
        if angular is None:
            angular = current_angular
        pyb.resetBaseVelocity(
            self.uid,
            list(linear),
            list(angular),
            physicsClientId=self.client_id,
        )

    def apply_wrench(
        self, force=None, torque=None, position=None, frame=pyb.LINK_FRAME
    ):
        """Apply a wrench (i.e., force and torque) to the body about its
        inertial frame.

        Parameters
        ----------
        force : iterable
            Force to apply to the body. If not provided, no force is applied.
        torque : iterable
            Torque to apply to the body. If not provided, no torque is applied.
        position : iterable
            Position at which to apply the force. If not provided, force acts
            at the origin of the specified frame. Has no effect on the torque.
        frame : int
            The coordinate frame (i.e., orientation). Can be either
            ``pyb.LINK_FRAME`` (the default) or ``pyb.WORLD_FRAME``.
        """
        if position is None:
            position = np.zeros(3)

        if force is not None:
            pyb.applyExternalForce(
                self.uid,
                -1,
                forceObj=list(force),
                posObj=list(position),
                flags=frame,
                physicsClientId=self.client_id,
            )
        if torque is not None:
            pyb.applyExternalTorque(
                self.uid,
                -1,
                torqueObj=list(torque),
                flags=frame,
                physicsClientId=self.client_id,
            )
