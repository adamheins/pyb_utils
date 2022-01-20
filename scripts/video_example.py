#!/usr/bin/env python
"""Example demonstrating camera utilities."""
import pybullet as pyb
import pybullet_data
import cv2

from pyb_utils.camera import Camera


class VideoRecorder:
    """Recorder for a video of a PyBullet simulation."""
    def __init__(self, filename, camera, fps):
        """Initialize the VideoRecorder.

        Parameters:
            filename: The file to write the video to.
            camera: Camera object to use for rendering frames.
            fps: Frames per second. Each `fps` frames will be played over one
                second of video.
        """
        self.camera = camera
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.writer = cv2.VideoWriter(
            filename,
            fourcc,
            fps,
            (camera.width, camera.height),
        )

    def capture_frame(self, rgba=None):
        """Capture a frame and write it to the video.

        Parameters:
            rgba: If provided, write this data to the video (this can be used
            to avoid multiple renderings with the camera). Otherwise, get the
            frame data from the camera.
        """
        if not rgba:
            rgba, _, _ = self.camera.get_frame()
        bgr = rgba[..., [2, 1, 0]]
        self.writer.write(bgr)


def load_environment(client_id):
    pyb.setAdditionalSearchPath(
        pybullet_data.getDataPath(), physicsClientId=client_id
    )

    # ground plane
    ground_id = pyb.loadURDF(
        "plane.urdf", [0, 0, 0], useFixedBase=True, physicsClientId=client_id
    )

    # KUKA iiwa robot arm
    kuka_id = pyb.loadURDF(
        "kuka_iiwa/model.urdf",
        [0, 0, 0],
        useFixedBase=True,
        physicsClientId=client_id,
    )

    bodies = {
        "robot": kuka_id,
        "ground": ground_id,
    }
    return bodies


def main():
    gui_id = pyb.connect(pyb.GUI)
    bodies = load_environment(gui_id)

    camera = Camera(
        camera_position=(1, 0, 1),
        target_position=(0, 0, 1),
        near=0.1,
        far=5,
        width=200,
        height=200,
    )

    video = VideoRecorder("test.mp4", camera, fps=10)

    for i in range(100):
        video.capture_frame()


if __name__ == "__main__":
    main()
