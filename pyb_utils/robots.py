import numpy as np
import pybullet as pyb


class Robot:
    """Wrapper for a PyBullet robot."""

    def __init__(self, uid):
        """Initialize using the uid of the robot."""
        self.uid = uid
        self.num_joints = pyb.getNumJoints(uid)
        self._joint_indices = list(range(self.num_joints))

    def get_joint_states(self):
        """Get position and velocity of the robot's joints."""
        states = pyb.getJointStates(self.uid, self._joint_indices)
        q = np.array([state[0] for state in states])
        v = np.array([state[1] for state in states])
        return q, v

    def command_velocity(self, u):
        """Send a joint velocity command to the robot."""
        pyb.setJointMotorControlArray(
            self.uid,
            self._joint_indices,
            controlMode=pyb.VELOCITY_CONTROL,
            targetVelocities=list(u),
        )
