"""This module provides a simple wrapper for a velocity- or torque-controlled
robot."""
import numpy as np
import pybullet as pyb

from .named_tuples import getJointInfo, getJointStates, getLinkState
from .math import quaternion_to_matrix


class Robot:
    """Wrapper for a PyBullet robot.

    Attributes
    ----------
    uid : int
        The unique ID of the underlying PyBullet robot body.
    num_joints : int
        The total number of joints (only moveable joints are included; fixed
        joints are not).
    num_actuated_joints : int
        The number of actuated (i.e., controlled) joints. This should never be
        more than `num_joints`.
    tool_idx : int
        The index of the tool joint. Note that this corresponds to the tool
        link, since each link has the same index as its parent joint.

    Parameters
    ----------
    uid : int
        The UID of the body representing the robot.
    tool_joint_name : str
        If provided, use the child link of this joint as the end effector. If
        it is ``None``, then the link corresponding to the last joint is used.
    actuated_joints : iterable
        An optional list of actuated joint names. These joints will be
        "actuated", in that they take commands from the ``command_`` methods.
        If not provided, all moveable joints are considered actuated. To have a
        free-moving joint (whether actuated or not), see
        :meth:`set_joint_friction_forces`.
    """

    def __init__(
        self,
        uid,
        tool_joint_name=None,
        actuated_joints=None,
    ):
        self.uid = uid

        n = pyb.getNumJoints(uid)

        # build a dict of all joint info
        joint_info = {}
        for i in range(n):
            info = getJointInfo(uid, i, decode="utf-8")
            joint_info[info.jointName] = info

        # record indices of all non-fixed joints
        self._joint_indices = []
        for name in joint_info:
            info = joint_info[name]
            if info.jointType == pyb.JOINT_FIXED:
                continue
            self._joint_indices.append(info.jointIndex)
        self.num_joints = len(self._joint_indices)

        if actuated_joints is None:
            self._actuated_joint_indices = self._joint_indices
        else:
            self._actuated_joint_indices = [
                joint_info[name].jointIndex for name in actuated_joints
            ]
        self.num_actuated_joints = len(self._actuated_joint_indices)

        if tool_joint_name is None:
            self.tool_idx = n - 1
        else:
            try:
                self.tool_idx = joint_info[tool_joint_name].jointIndex
            except KeyError:
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

    def set_joint_friction_forces(self, forces, joint_indices=None):
        """Set the friction forces at each joint.

        Set the force to a small value (or zero) to have free-moving joints.

        Parameters
        ----------
        forces : iterable
            The friction forces for the specified joints.
        joint_indices : iterable
            An optional list of joint indices to specify forces for. If not
            provided, then it is assumed forces are being provided for all
            joints.
        """

        if joint_indices is None:
            joint_indices = self._joint_indices

        assert len(forces) == len(
            joint_indices
        ), f"Number of friction forces does not match number of joints."

        pyb.setJointMotorControlArray(
            self.uid,
            joint_indices,
            controlMode=pyb.VELOCITY_CONTROL,
            targetVelocities=list(np.zeros_like(forces)),
            forces=forces,
        )

    def command_velocity(self, u):
        """Send a joint velocity command to the robot.

        Parameters
        ----------
        u : iterable
            The joint velocity to command. Must be of length
            ``self.num_actuated_joints``.
        """
        assert (
            len(u) == self.num_actuated_joints
        ), f"Command length does not match number of actuated joints."
        pyb.setJointMotorControlArray(
            self.uid,
            self._actuated_joint_indices,
            controlMode=pyb.VELOCITY_CONTROL,
            targetVelocities=list(u),
        )

    def command_effort(self, u):
        """Send a joint effort (force/torque) command to the robot.

        Parameters
        ----------
        u : iterable
            The joint effort to command. Must be of length
            ``self.num_actuated_joints``.
        """
        assert (
            len(u) == self.num_actuated_joints
        ), f"Command length does not match number of actuated joints."

        pyb.setJointMotorControlArray(
            self.uid,
            self._actuated_joint_indices,
            controlMode=pyb.TORQUE_CONTROL,
            forces=list(u),
        )

    def get_link_com_pose(self, link_idx=None, as_rotation_matrix=False):
        """Get the pose of a link's center of mass.

        The pose is computed about the link's center of mass with respect to
        the world frame and expressed in the world frame.

        Let :math:`\\mathcal{F}_w` be the world frame, :math:`\\mathcal{F}_f` be
        the link's URDF frame, and let :math:`\\mathcal{F}_c` be the link's CoM
        frame. This function returns the position :math:`\\boldsymbol{r}^{cw}_w` and
        orientation quaternion :math:`\\boldsymbol{Q}_{wc}`. The relationship
        between :math:`(\\boldsymbol{r}^{cw}_w,\\boldsymbol{Q}_{wc})` from this function
        and :math:`(\\boldsymbol{r}^{fw}_w,\\boldsymbol{Q}_{wf})` from
        :meth:`get_link_frame_pose` is

        .. math::
           \\boldsymbol{r}^{cw}_w &= \\boldsymbol{r}^{fw}_w + \\boldsymbol{C}_{wf}\\boldsymbol{r}^{cf}_f \\\\
           \\boldsymbol{Q}_{wc} &= \\boldsymbol{Q}_{wf} \\otimes \\boldsymbol{Q}_{fc},

        where :math:`\\boldsymbol{C}_{wf}` is the rotation matrix representing the
        same rotation as :math:`\\boldsymbol{Q}_{wf}`, and :math:`\\boldsymbol{r}^{cf}_f`
        and :math:`\\boldsymbol{Q}_{fc}` are the position and quaternion from
        ``LinkState.localInertialFramePosition`` and
        ``LinkState.localInertialFrameOrientation``, respectively, obtained
        from a call to :func:`pyb_utils.named_tuples.getLinkState`. The symbol
        :math:`\\otimes` refers to Hamilton/quaternion multiplication.

        Parameters
        ----------
        link_idx :
            Index of the link to use. If not provided, defaults to the end
            effector ``self.tool_idx``.
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
        if link_idx is None:
            link_idx = self.tool_idx
        state = getLinkState(self.uid, link_idx, computeForwardKinematics=True)
        position = np.array(state.linkWorldPosition)
        orientation = np.array(state.linkWorldOrientation)
        if as_rotation_matrix:
            orientation = quaternion_to_matrix(orientation)
        return position, orientation

    def get_link_frame_pose(self, link_idx=None, as_rotation_matrix=False):
        """Get the pose of a link's URDF frame.

        The pose is computed about the link's parent joint position, which is
        its URDF frame.

        Parameters
        ----------
        link_idx :
            Index of the link to use. If not provided, defaults to the end
            effector ``self.tool_idx``.
        as_rotation_matrix : bool
            Set to ``True`` to return the orientation as a rotation matrix,
            ``False`` to return a quaternion.

        Returns
        -------
        :
            A tuple containing the position and orientation of the link's frame
            with respect to the world. If ``as_rotation_matrix=True``, then the
            orientation is represented as a :math:`3\\times3` rotation matrix.
            If ``False``, then it is represented as a quaternion in xyzw order.
        """
        if link_idx is None:
            link_idx = self.tool_idx
        state = getLinkState(self.uid, link_idx, computeForwardKinematics=True)
        position = np.array(state.worldLinkFramePosition)
        orientation = np.array(state.worldLinkFrameOrientation)
        if as_rotation_matrix:
            orientation = quaternion_to_matrix(orientation)
        return position, orientation

    def get_link_com_velocity(self, link_idx=None):
        """Get the velocity of a link's center of mass with respect to the world.

        With reference to the documentation of :meth:`get_link_com_pose`, the
        relationship between the CoM velocity
        :math:`(\\boldsymbol{v}^{cw}_w,\\boldsymbol{\\omega}^{cw}_w)` from this
        function and
        :math:`(\\boldsymbol{v}^{fw}_w,\\boldsymbol{\\omega}^{fw}_w)` from
        :meth:`get_link_frame_velocity` is

        .. math::
           \\boldsymbol{v}^{cw}_w &= \\boldsymbol{v}^{fw}_w + \\boldsymbol{\\omega}^{fw}_w\\times\\boldsymbol{C}_{wf}\\boldsymbol{r}^{cf}_f \\\\
           \\boldsymbol{\\omega}^{cw}_w &= \\boldsymbol{\\omega}^{fw}_w,

        where :math:`\\times` denotes the cross product.


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
