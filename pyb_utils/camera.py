"""This module provides a utility ``Camera`` class for PyBullet."""
from pathlib import Path

import numpy as np
import pybullet as pyb
from PIL import Image
import cv2


class Camera:
    """A PyBullet camera, which captures images of the simulation environment.

    Parameters
    ----------
    view_matrix : iterable
        The camera's view matrix.
    near : float
        Near value
    far : float
        Far value
    fov : float
        Field of view
    width : int
        Width of the image in pixels
    height : int
        Height of the image in pixels
    """

    def __init__(
        self,
        view_matrix,
        near=0.1,
        far=1000.0,
        fov=60.0,
        width=1280,
        height=720,
    ):
        self.near = near
        self.far = far
        self.width = width
        self.height = height

        self.view_matrix = view_matrix
        self.proj_matrix = pyb.computeProjectionMatrixFOV(
            fov=fov, aspect=width / height, nearVal=near, farVal=far
        )

    @classmethod
    def from_camera_position(
        cls,
        target_position,
        camera_position,
        near=0.1,
        far=1000.0,
        fov=60.0,
        width=1280,
        height=720,
    ):
        """Construct a new camera from target and camera positions.

        Parameters
        ----------
        target_position : iterable
            The position of the camera's target point.
        camera_position : iterable
            The position of the camera in the world.
        near : float
            Near value
        far : float
            Far value
        fov : float
            Field of view
        width : int
            Width of the image in pixels
        height : int
            Height of the image in pixels
        """
        view_matrix = pyb.computeViewMatrix(
            cameraEyePosition=camera_position,
            cameraTargetPosition=target_position,
            cameraUpVector=[0, 0, 1],
        )
        return cls(
            view_matrix=view_matrix,
            near=near,
            far=far,
            fov=fov,
            width=width,
            height=height,
        )

    @classmethod
    def from_distance_rpy(
        cls,
        target_position,
        distance,
        roll=0,
        pitch=0,
        yaw=0,
        near=0.1,
        far=1000.0,
        fov=60.0,
        width=1280,
        height=720,
    ):
        """Construct a new camera from target position, distance, and roll,
        pitch, yaw angles.

        Parameters
        ----------
        target_position : iterable
            The position of the camera's target point.
        distance : float
            The distance of camera from target.
        roll : float
            Roll of the camera's orientation.
        pitch : float
            Pitch of the camera's orientation.
        yaw : float
            Yaw of the camera's orientation.
        near : float
            Near value
        far : float
            Far value
        fov : float
            Field of view
        width : int
            Width of the image in pixels
        height : int
            Height of the image in pixels
        """
        view_matrix = pyb.computeViewMatrixFromYawPitchRoll(
            distance=distance,
            yaw=yaw,
            pitch=pitch,
            roll=roll,
            cameraTargetPosition=target_position,
            upAxisIndex=2,
        )
        return cls(
            view_matrix=view_matrix,
            near=near,
            far=far,
            fov=fov,
            width=width,
            height=height,
        )

    def get_frame(self):
        """Get a frame from the camera.

        Returns
        -------
        :
            A tuple ``(rgba, depth, seg)``, where ``rgba`` is the RGBA color
            data of shape ``(height, width, 4)``, ``depth`` is the depth buffer
            of shape ``(height, width)``, and ``seg`` is the segmentation mask
            of shape ``(height, width)``.
        """
        _, _, rgba, depth, seg = pyb.getCameraImage(
            width=self.width,
            height=self.height,
            shadow=1,
            viewMatrix=self.view_matrix,
            projectionMatrix=self.proj_matrix,
            renderer=pyb.ER_BULLET_HARDWARE_OPENGL,
        )

        # the image has height rows x width columns
        return (
            np.array(rgba, dtype=np.uint8).reshape(
                (self.height, self.width, 4)
            ),
            np.array(depth).reshape((self.height, self.width)),
            np.array(seg).reshape((self.height, self.width)),
        )

    def save_frame(self, filename, rgba=None):
        """Save a frame to a file.

        Parameters
        ----------
        filename : str
            The name of the image file.
        rgba : np.ndarray
            Optionally, the user can provide the RGBA data from a previous call
            to :meth:`Camera.get_frame()`. If not provided,
            ``self.get_frame()`` is called to retrieve this data.
        """
        if rgba is None:
            rgba, _, _ = self.get_frame()
        Image.fromarray(rgba, "RGBA").save(filename)

    def linearize_depth(self, depth=None):
        """Convert depth map to actual distance from camera plane.

        See https://stackoverflow.com/a/6657284.

        Parameters
        ----------
        depth : np.ndarray
            Optionally, the user can provide the depth buffer from a previous
            call to :meth:`Camera.get_frame()`. If not provided,
            ``self.get_frame()`` is called to retrieve this data.

        Returns
        -------
        :
            Linearized depth buffer, consisting of the actual depth values from
            the camera plane.
        """
        if depth is None:
            _, depth, _ = self.get_frame()
        depth_ndc = 2 * depth - 1  # normalized device coordinates
        depth_linear = (
            2.0
            * self.near
            * self.far
            / (self.far + self.near - depth_ndc * (self.far - self.near))
        )
        return depth_linear

    def set_camera_pose(self, position, target):
        """Change the position and target of the camera.

        Parameters
        ----------
        position : iterable
            The position of the camera in the world.
        target : iterable
            The position of the camera's target point.
        """
        self.position = position
        self.target = target
        self.view_matrix = pyb.computeViewMatrix(
            cameraEyePosition=position,
            cameraTargetPosition=target,
            cameraUpVector=[0, 0, 1],
        )

    def get_point_cloud(self, depth=None):
        """Convert depth buffer to 3D point cloud in world coordinates.

        See https://stackoverflow.com/a/62247245 for the main source of this
        code.

        Parameters
        ----------
        depth : np.ndarray
            Optionally, the user can provide the depth buffer from a previous
            call to :meth:`Camera.get_frame()`. If not provided,
            ``self.get_frame()`` is called to retrieve this data.

        Returns
        -------
        An array of shape ``(height, width, 3)`` representing the points seen
        by the camera.
        """
        if depth is None:
            _, depth, _ = self.get_frame()

        # view matrix maps world coordinates to camera coordinates (extrinsics)
        V = np.array(self.view_matrix).reshape((4, 4), order="F")

        # camera projection matrix: map camera coordinates to clip coordinates
        # (intrinsics)
        P = np.array(self.proj_matrix).reshape((4, 4), order="F")

        PV_inv = np.linalg.inv(P @ V)

        points = np.zeros((self.height, self.width, 3))
        for h in range(self.height):
            for w in range(self.width):
                # convert to normalized device coordinates
                # notice that the y-transform is negative---we actually have a
                # left-handed coordinate frame here (x = right, y = down, z =
                # out of the screen)
                x = (2 * w - self.width) / self.width
                y = -(2 * h - self.height) / self.height

                # depth buffer is already in range [0, 1]
                z = 2 * depth[h, w] - 1

                # back out to world coordinates by applying inverted projection
                # and view matrices
                r_ndc = np.array([x, y, z, 1])
                r_world_unnormalized = PV_inv @ r_ndc

                # normalize homogenous coordinates to get rid of perspective
                # divide
                points[h, w, :] = (
                    r_world_unnormalized[:3] / r_world_unnormalized[3]
                )
        return points


class VideoRecorder:
    """Recorder for a video of a PyBullet simulation.

    Parameters
    ----------
    filename : str
        The name of the file to write the video to.
    camera : Camera
        Camera object to use for rendering frames.
    fps : int
        Frames per second. Each ``fps`` frames will be played over one second
        of video.
    """

    def __init__(self, filename, camera, fps, codec="mp4v"):
        self.camera = camera
        fourcc = cv2.VideoWriter_fourcc(*codec)
        self.writer = cv2.VideoWriter(
            str(filename),
            fourcc,
            fps,
            (camera.width, camera.height),
        )

    def capture_frame(self, rgba=None):
        """Capture a frame and write it to the video.

        Parameters
        ----------
        rgba : np.ndarray
            Optionally, the user can provide the RGBA data from a previous call
            to :meth:`Camera.get_frame()`. This can be used to avoid multiple
            renderings with the camera. If not provided,
            ``self.camera.get_frame()`` is called to retrieve this data.
        """
        if rgba is None:
            rgba, _, _ = self.camera.get_frame()

        # OpenCV uses BGR instead of RGB
        bgr = rgba[..., [2, 1, 0]]
        self.writer.write(bgr)


class FrameRecorder:
    """Recorder for a set of frames from a PyBullet simulation.

    This is an alternative to the :class:`VideoRecorder`, which directly saves
    a video.

    Parameters
    ----------
    camera : Camera
        Camera object to use for rendering frames.
    fps : int
        Frames per second. Each ``fps`` frames will be played over one second
        of video.
    dirname : str
        The name of the directory to write the frames to. The directory will be
        created if it doesn't already exist.
    """
    def __init__(self, camera, fps=25, dirname=None):
        self.camera = camera
        self.timestep = 1.0 / fps
        self.save = not (dirname is None)

        if self.save:
            self.path = Path(dirname)
            self.path.mkdir(exist_ok=True)

        self.frame_count = 0
        self.last_record_time = -np.inf

    def capture_frame(self, t, rgba=None):
        """Capture and save a frame if enough time has past since the last
        capture.

        This does nothing if less than ``1. / fps`` time has elapsed since the
        last frame was captured.

        Parameters
        ----------
        t : float
            The current time.
        rgba :
            Image data as returned by :meth:`Camera.get_frame`. Otherwise,
            ``self.camera.get_frame`` is called to retrieve this data.
        """
        if not self.save:
            return

        # bail if not enough time has elapsed to record a new frame
        if t < self.last_record_time + self.timestep:
            return

        # capture and save the frame
        frame_path = self.path / f"frame_{self.frame_count}.png"
        self.camera.save_frame(frame_path, rgba=rgba)

        self.frame_count += 1
        self.last_record_time = t
