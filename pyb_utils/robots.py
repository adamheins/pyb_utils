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
    client_id : int
        The unique ID of the physics server this robot belongs to.
    tool_idx : int
        The index of the tool joint/link (in PyBullet each link has the same
        index as its parent joint).

    Parameters
    ----------
    uid : int
        The UID of the body representing the robot.
    tool_link_name : str
        If provided, use this link as the end effector. If it is ``None``, then
        the last link is used. The index of this link is stored in
        ``self.tool_idx``.
    actuated_joint_names : iterable
        An optional list of actuated joint names. These joints will be
        "actuated", in that they take commands from the ``command_`` methods.
        If not provided, all moveable joints are considered actuated. To have a
        free-moving joint (whether actuated or not), see
        :meth:`set_joint_friction_forces`.
    client_id : int
        The physics server UID to connect to, if multiple servers are being
        used.

    Raises
    ------
    ValueError
        If ``tool_link_name`` is provided but the robot has no link with that
        name.
    """

    def __init__(
        self,
        uid,
        tool_link_name=None,
        actuated_joint_names=None,
        client_id=0,
    ):
        self.uid = uid
        self.client_id = client_id

        n = pyb.getNumJoints(uid, physicsClientId=client_id)

        # build mappings of joint and link names to their respective indices
        # for look up later
        # also record the indices of the moveable joints
        self._link_index_map = {}
        self._joint_index_map = {}
        self._moveable_joint_indices = []
        for i in range(n):
            info = getJointInfo(
                uid, i, decode="utf-8", physicsClientId=client_id
            )
            self._joint_index_map[info.jointName] = i
            self._link_index_map[info.linkName] = i

            if info.jointType != pyb.JOINT_FIXED:
                self._moveable_joint_indices.append(i)

        if actuated_joint_names is None:
            self._actuated_joint_indices = self._moveable_joint_indices
        else:
            self._actuated_joint_indices = [
                self._joint_index_map[name] for name in actuated_joint_names
            ]

        if tool_link_name is None:
            self.tool_idx = n - 1
        else:
            self.tool_idx = self.get_link_index(tool_link_name)

    @property
    def num_total_joints(self):
        """int: The total number of joints, including fixed and moveable joints."""
        return len(self._joint_index_map)

    @property
    def num_moveable_joints(self):
        """int: The number of moveable joints."""
        return len(self._moveable_joint_indices)

    @property
    def num_actuated_joints(self):
        """int: The number of actuated (i.e., controlled) joints. This should
        never be more than `num_joints`."""
        return len(self._actuated_joint_indices)

    @property
    def link_names(self):
        """list: A list of all link names."""
        return list(self._link_index_map.keys())

    @property
    def all_joint_names(self):
        """list: A list of all joint names, including fixed and moveable joints."""
        return list(self._joint_index_map.keys())

    @property
    def moveable_joint_names(self):
        """list: A list of the names of the moveable joints."""
        return [self.all_joint_names[i] for i in self._moveable_joint_indices]

    @property
    def actuated_joint_names(self):
        """list: A list of the actuated joint names. This is the same as the
        list of joint names unless ``actuated_joint_names`` is provided to the
        constructor."""
        return [self.all_joint_names[i] for i in self._actuated_joint_indices]

    def get_joint_index(self, name):
        """Get the index of the joint named ``name``.

        Parameters
        ----------
        name : str
            The name of the joint.

        Returns
        -------
        :
            The index of the joint.

        Raises
        ------
        ValueError
            If the robot has no joint named ``name``.
        """
        try:
            return self._joint_index_map[name]
        except KeyError:
            raise ValueError(f"Robot has no joint named {name}.")

    def get_link_index(self, name):
        """Get the index of the link named ``name``.

        Parameters
        ----------
        name : str
            The name of the link.

        Returns
        -------
        :
            The index of the link.

        Raises
        ------
        ValueError
            If the robot has no link named ``name``.
        """
        try:
            return self._link_index_map[name]
        except KeyError:
            raise ValueError(f"Robot has no link named {name}.")

    def get_joint_states(self):
        """Get position and velocity of the robot's moveable joints.

        Returns
        -------
        :
            A tuple ``(q, v)`` where ``q`` is the robot's joint configuration
            and ``v`` is the joint velocity.
        """
        states = getJointStates(
            self.uid,
            self._moveable_joint_indices,
            physicsClientId=self.client_id,
        )
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
            ``self.num_moveable_joints``.
        """
        for idx, angle in zip(self._moveable_joint_indices, q):
            pyb.resetJointState(
                self.uid, idx, angle, physicsClientId=self.client_id
            )

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
            moveable joints.
        """

        if joint_indices is None:
            joint_indices = self._moveable_joint_indices

        assert len(forces) == len(
            joint_indices
        ), f"Number of friction forces does not match number of joints."

        pyb.setJointMotorControlArray(
            self.uid,
            joint_indices,
            controlMode=pyb.VELOCITY_CONTROL,
            targetVelocities=list(np.zeros_like(forces)),
            forces=forces,
            physicsClientId=self.client_id,
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
            physicsClientId=self.client_id,
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
            physicsClientId=self.client_id,
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
        state = getLinkState(
            self.uid,
            link_idx,
            computeForwardKinematics=True,
            physicsClientId=self.client_id,
        )
        position = np.array(state.linkWorldPosition)
        orientation = np.array(state.linkWorldOrientation)
        orientation /= np.linalg.norm(orientation)
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
        state = getLinkState(
            self.uid,
            link_idx,
            computeForwardKinematics=True,
            physicsClientId=self.client_id,
        )
        position = np.array(state.worldLinkFramePosition)
        orientation = np.array(state.worldLinkFrameOrientation)
        orientation /= np.linalg.norm(orientation)
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
            physicsClientId=self.client_id,
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
            physicsClientId=self.client_id,
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

        Jv, Jw = pyb.calculateJacobian(
            self.uid, link_idx, offset, q, z, z, physicsClientId=self.client_id
        )
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
        state = getLinkState(self.uid, link_idx, physicsClientId=self.client_id)
        return self.compute_link_jacobian(
            q=q, link_idx=link_idx, offset=state.localInertialFramePosition
        )
