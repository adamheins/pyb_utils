"""Provides a utility Camera class for PyBullet."""
import numpy as np
import pybullet as pyb
from PIL import Image


class Camera:
    """A PyBullet camera."""

    def __init__(
        self,
        camera_position,
        target_position,
        near=0.1,
        far=1000.0,
        fov=60.0,
        width=1280,
        height=720,
    ):
        """Initialize the camera.

        Parameters:
            camera_position: The position of the camera in the world
            target_position: The position of the camera's target point
            near: Near value
            far: Far value
            fov: Field of view
            width: Width of the image
            height: Height of the image
        """
        self.position = camera_position
        self.target = target_position
        self.near = near
        self.far = far
        self.width = width
        self.height = height

        self.view_matrix = pyb.computeViewMatrix(
            cameraEyePosition=camera_position,
            cameraTargetPosition=target_position,
            cameraUpVector=[0, 0, 1],
        )
        self.proj_matrix = pyb.computeProjectionMatrixFOV(
            fov=fov, aspect=width / height, nearVal=near, farVal=far
        )

    def get_frame(self):
        """Get a frame from the camera.

        Returns:
            rgba: The RGBA colour data
            depth: Depth buffer
            seg: Segmentation mask
        """
        _, _, rgba, depth, seg = pyb.getCameraImage(
            width=self.width,
            height=self.height,
            shadow=1,
            viewMatrix=self.view_matrix,
            projectionMatrix=self.proj_matrix,
            renderer=pyb.ER_BULLET_HARDWARE_OPENGL,
        )
        return rgba, depth, seg

    def save_frame(self, filename, rgba=None):
        """Save a frame to a file.

        Parameters:
            filename: The name of the image file
            rgba: Optional, RGBA data provided by `Camera.get_frame()`. If not
                provided, `self.get_frame()` is called to retrieve this data.
        """
        if rgba is None:
            rgba, _, _ = self.get_frame()
        img = Image.fromarray(
            np.reshape(rgba, (self.height, self.width, 4)), "RGBA"
        )
        img.save(filename)

    def linearize_depth(self, depth=None):
        """Convert depth map to actual distance from camera plane.

        See <https://stackoverflow.com/a/6657284>.

        Parameters:
            depth: Optional, depth buffer provided by `Camera.get_frame()`. If
                not provided, `self.get_frame()` is called to retrieve this
                data.

        Returns: linearized depth buffer: actual depth values from the camera
            plane
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
        """Change position and target of the camera."""
        self.position = position
        self.target = target
        self.view_matrix = pyb.computeViewMatrix(
            cameraEyePosition=position,
            cameraTargetPosition=target,
            cameraUpVector=[0, 0, 1],
        )

    def get_point_cloud(self, depth=None):
        """Convert depth buffer to 3D point cloud in world coordinates.

        See <https://stackoverflow.com/a/62247245> for the main source of this
        code.

        Parameters:
            depth: Optional, depth buffer provided by `Camera.get_frame()`. If
                not provided, `self.get_frame()` is called to retrieve this
                data.

        Returns: A (width, height, 3)-dimensional array of points seen by the
            camera.
        """
        if depth is None:
            _, depth, _ = self.get_frame()

        # view matrix maps world coordinates to camera coordinates (extrinsics)
        V = np.array(self.view_matrix).reshape((4, 4), order="F")

        # camera projection matrix: map camera coordinates to clip coordinates
        # (intrinsics)
        P = np.array(self.proj_matrix).reshape((4, 4), order="F")

        PV_inv = np.linalg.inv(P @ V)

        # depth is stored (height * width) (i.e., transpose of what one might
        # expect on the numpy side)
        points = np.zeros((self.width, self.height, 3))
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
                points[w, h, :] = (
                    r_world_unnormalized[:3] / r_world_unnormalized[3]
                )
        return points
