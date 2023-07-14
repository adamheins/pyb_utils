import pybullet as pyb
import numpy as np


class BulletBody:
    """Generic rigid body in PyBullet."""

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
        """
        Parameters:
            position: iterable of length 3; position in the world.
            collision_uid: ID of the collision shape to use.
            visual_uid: ID of the visual shape to use.
            mass: mass of the body; defaults to 1.
            orientation: iterable of length 4 representing a quaternion (x, y, z, w);
                if not provided, orientation is aligned with the world frame axes.
            client_id: physics client ID; only required if connected to
                multiple servers.
        """
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

        Parameters:
            position: iterable of length 3; position in the world.
            half_extents: iterable of length 3; half lengths of the body.
            color: 4-tuple (r, g, b, α).
            client_id: physics client ID; only required if connected to
                multiple servers.
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

        Parameters:
            position: iterable of length 3; position in the world.
            radius: radius of the sphere.
            color: 4-tuple (r, g, b, α).
            client_id: physics client ID; only required if connected to
                multiple servers.
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

        Parameters:
            position: iterable of length 3; position in the world.
            radius: radius of the cylinder.
            height: the height of the cylinder.
            color: 4-tuple (r, g, b, α).
            client_id: physics client ID; only required if connected to
                multiple servers.
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

        Parameters:
            position: iterable of length 3; position in the world.
            radius: radius of the capsule.
            height: the height of the capsule.
            color: 4-tuple (r, g, b, α).
            client_id: physics client ID; only required if connected to
                multiple servers.
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

    def get_pose(self):
        """Get the position and orientation of the body.

        Returns a tuple (position, orientation), where the position is of array
        of length 3 and orientation is an array of length 4 representing a
        quaternion (x, y, z, w).
        """
        pos, orn = pyb.getBasePositionAndOrientation(
            self.uid, physicsClientId=self.client_id
        )
        return np.array(pos), np.array(orn)

    def get_velocity(self):
        """Get the velocity of the body.

        Returns a tuple (linear, angular) containing the linear and angular
        velocity, where each is an array of length 3.
        """
        linear, angular = pyb.getBaseVelocity(
            self.uid, physicsClientId=self.client_id
        )
        return np.array(linear), np.array(angular)

    def set_pose(self, position=None, orientation=None):
        """Set the position and orientation of the body.

        Parameters:
            position: iterable of length 3; if not provided, position is unchanged.
            orientation: iterable of length 4 representing a quaternion (x, y,
                z, w); if not provided, orientation is unchanged.
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
        """Set the velocity of the body.

        Parameters:
            linear: iterable of length 3; if not provided, linear velocity is unchanged.
            angular: iterable of length 3; if not provided, angular velocity is unchanged.
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
        """Apply a wrench (i.e., force and torque) to the body.

        Paramters:
            force: iterable of length 3; if not provided, no force is applied.
            torque: iterable of length 3; if not provided, no torque is applied.
            position: iterable of length 3; if not provided, force acts at the
                origin of the specified frame. Has no effect on the torque.
            frame: int representing the coordinate frame. Can be either
                pyb.LINK_FRAME (the default) or pyb.WORLD_FRAME.
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
