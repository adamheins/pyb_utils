"""This module provides a simple wrapper for a velocity-controlled robot."""
import numpy as np
import pybullet as pyb

from .named_tuples import getJointInfo, getJointStates, getLinkState
from .math import quaternion_to_matrix


class Robot:
    """Wrapper for a PyBullet robot.

    Parameters
    ----------
    uid : int
        The UID of the body representing the robot.
    tool_joint_name :
        If supplied, use the child link of this joint as the end effector. If
        it is ``None``, then the link corresponding to the last joint is used.
    """

    def __init__(self, uid, tool_joint_name=None):
        self.uid = uid
        self.num_joints = pyb.getNumJoints(uid)
        self._joint_indices = list(range(self.num_joints))

        if tool_joint_name is None:
            self.tool_idx = self._joint_indices[-1]
        else:
            self.tool_idx = None
            for i in range(self.num_joints):
                info = getJointInfo(self.uid, i, decode="utf-8")
                if info.jointName == tool_joint_name:
                    self.tool_idx = info.jointIndex
                    break
            if self.tool_idx is None:
                raise ValueError(f"No joint with name {tool_joint_name} found.")

    def get_joint_states(self):
        """Get position and velocity of the robot's joints.

        Returns
        -------
        :
            A tuple ``(q, v)`` where ``q`` is the robot's joint configuration
            and ``v`` is the joint velocity.
        """
        states = getJointStates(self.uid, self._joint_indices)
        q = np.array([state.jointPosition for state in states])
        v = np.array([state.jointVelocity for state in states])
        return q, v

    def reset_joint_configuration(self, q):
        """Reset the robot to a particular configuration.

        It is best not to do this during a simulation, as it overrides all
        dynamic effects.

        Parameters
        ----------
        q : iterable
            The vector of joint values to set. Must be of length
            ``self.num_joints``.
        """
        for idx, angle in zip(self._joint_indices, q):
            pyb.resetJointState(self.uid, idx, angle)

    def command_velocity(self, u):
        """Send a joint velocity command to the robot.

        Parameters
        ----------
        u : iterable
            The joint velocity to command. Must be of length
            ``self.num_joints``.
        """
        pyb.setJointMotorControlArray(
            self.uid,
            self._joint_indices,
            controlMode=pyb.VELOCITY_CONTROL,
            targetVelocities=list(u),
        )

    def get_link_com_pose(self, link_idx=None):
        """Get the pose of a link's center of mass.

        The pose is computed about the link's center of mass with respect to
        the world frame.

        Parameters
        ----------
        link_idx :
            Index of the link to use. If not provided, defaults to the end
            effector ``self.tool_idx``.

        Returns
        -------
        :
            A tuple containing the position and orientation quaternion of the
            link's center of mass in the world frame. The quaternion is
            represented in xyzw order.
        """
        if link_idx is None:
            link_idx = self.tool_idx
        state = getLinkState(self.uid, link_idx, computeForwardKinematics=True)
        return np.array(state.linkWorldPosition), np.array(
            state.linkWorldOrientation
        )

    def get_link_frame_pose(self, link_idx=None):
        """Get the pose of a link's URDF frame.

        The pose is computed about the link's parent joint position, which is
        its URDF frame.

        Parameters
        ----------
        link_idx :
            Index of the link to use. If not provided, defaults to the end
            effector ``self.tool_idx``.

        Returns
        -------
        :
            A tuple containing the position and orientation quaternion of the
            link's frame with respect to the world. The quaternion is
            represented in xyzw order.
        """
        if link_idx is None:
            link_idx = self.tool_idx
        state = getLinkState(self.uid, link_idx, computeForwardKinematics=True)
        return np.array(state.worldLinkFramePosition), np.array(
            state.worldLinkFrameOrientation
        )

    def get_link_com_velocity(self, link_idx=None):
        """Get the velocity of a link's center of mass with respect to the world.

        Parameters
        ----------
        link_idx :
            Index of the link to use. If not provided, defaults to the end
            effector ``self.tool_idx``.

        Returns
        -------
        :
            A tuple containing the linear and angular velocity vectors for the
            link's center of mass in the world frame.
        """
        if link_idx is None:
            link_idx = self.tool_idx
        state = getLinkState(
            self.uid,
            link_idx,
            computeLinkVelocity=True,
        )
        v_com = np.array(state.worldLinkLinearVelocity)
        ω_com = np.array(state.worldLinkAngularVelocity)
        return v_com, ω_com

    def get_link_frame_velocity(self, link_idx=None):
        """Get the velocity of a link's URDF frame with respect to the world.

        Parameters
        ----------
        link_idx :
            Index of the link to use. If not provided, defaults to the end
            effector ``self.tool_idx``.

        Returns
        -------
        :
            A tuple containing the linear and angular velocity vectors for the
            link's URDF frame with respect to the world.
        """
        if link_idx is None:
            link_idx = self.tool_idx
        state = getLinkState(
            self.uid,
            link_idx,
            computeLinkVelocity=True,
            computeForwardKinematics=True,
        )
        C = quaternion_to_matrix(state.worldLinkFrameOrientation)
        v_com = np.array(state.worldLinkLinearVelocity)
        ω_com = np.array(state.worldLinkAngularVelocity)
        v = v_com - np.cross(ω_com, C @ state.localInertialFramePosition)
        ω = ω_com
        return v, ω

    def compute_link_jacobian(self, q=None, link_idx=None, offset=None):
        """Get the Jacobian of a point on a link at the given configuration.

        When ``offset`` is ``None`` or zeros, the Jacobian is computed around
        the URDF link frame (i.e., the parent joint position).

        See also
        https://github.com/bulletphysics/bullet3/issues/2429#issuecomment-538431246.

        Parameters
        ----------
        q :
            The joint configuration at which to compute the Jacobian. If no
            configuration is given, then the current one is used.
        link_idx :
            The index of the link to compute the Jacobian for. The end effector
            link ``self.tool_idx`` is used if not given.
        offset :
            Offset from the parent joint position at which to compute the
            Jacobian. Defaults to zero (no offset).

        Returns
        -------
        :
            The :math:`6\\times n` Jacobian matrix, where :math:`n` is the
            number of joints.
        """

        if q is None:
            q, _ = self.get_joint_states()
        if offset is None:
            offset = np.zeros(3)
        if link_idx is None:
            link_idx = self.tool_idx

        z = list(np.zeros_like(q))
        q = list(q)
        offset = list(offset)

        Jv, Jw = pyb.calculateJacobian(self.uid, link_idx, offset, q, z, z)
        J = np.vstack((Jv, Jw))
        return J

    def compute_link_frame_jacobian(self, q=None, link_idx=None):
        """Compute the Jacobian around the URDF link frame.

        This is the same as calling ``compute_link_jacobian`` with
        ``offset=None``.

        Parameters
        ----------
        q :
            The joint configuration at which to compute the Jacobian. If no
            configuration is given, then the current one is used.
        link_idx :
            The index of the link to compute the Jacobian for. The end effector
            link ``self.tool_idx`` is used if not given.

        Returns
        -------
        :
            The :math:`6\\times n` Jacobian matrix, where :math:`n` is the
            number of joints.
        """
        return self.compute_link_jacobian(q=q, link_idx=link_idx, offset=None)

    def compute_link_com_jacobian(self, q=None, link_idx=None):
        """Compute the Jacobian around the link's center of mass.

        Parameters
        ----------
        q :
            The joint configuration at which to compute the Jacobian. If no
            configuration is given, then the current one is used.
        link_idx :
            The index of the link to compute the Jacobian for. The end effector
            link ``self.tool_idx`` is used if not given.

        Returns
        -------
        :
            The :math:`6\\times n` Jacobian matrix, where :math:`n` is the
            number of joints.
        """
        if link_idx is None:
            link_idx = self.tool_idx
        state = getLinkState(self.uid, link_idx)
        return self.compute_link_jacobian(
            q=q, link_idx=link_idx, offset=state.localInertialFramePosition
        )
