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
        **kwargs
    ):
        if orientation is None:
            orientation = (0, 0, 0, 1)

        self.uid = pyb.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_uid,
            baseVisualShapeIndex=visual_uid,
            basePosition=tuple(position),
            baseOrientation=tuple(orientation),
            **kwargs,
        )

    @classmethod
    def box(cls, position, half_extents, color=(1, 0, 0, 1), **kwargs):
        collision_uid = pyb.createCollisionShape(
            shapeType=pyb.GEOM_BOX,
            halfExtents=tuple(half_extents),
        )
        visual_uid = pyb.createVisualShape(
            shapeType=pyb.GEOM_BOX,
            halfExtents=tuple(half_extents),
            rgbaColor=tuple(color),
        )
        return cls(position, collision_uid, visual_uid, **kwargs)

    @classmethod
    def sphere(cls):
        # TODO
        pass

    @classmethod
    def cylinder(cls):
        # TODO
        pass

    def get_pose(self):
        pos, orn = pyb.getBasePositionAndOrientation(self.uid)
        return np.array(pos), np.array(orn)

    def get_velocity(self):
        linear, angular = pyb.getBaseVelocity(self.uid)
        return np.array(linear), np.array(angular)

    def set_pose(self, position=None, orientation=None):
        current_pos, current_orn = self.get_pose()
        if position is None:
            position = current_pos
        if orientation is None:
            orientation = current_orn
        pyb.resetBasePositionAndOrientation(self.uid, list(position), list(orientation))

    def set_velocity(self, linear=None, angular=None):
        current_linear, current_angular = self.get_velocity()
        if linear is None:
            linear = current_linear
        if angular is None:
            angular = current_angular
        pyb.resetBaseVelocity(self.uid, linear, angular)
