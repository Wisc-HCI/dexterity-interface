from sensor_interface.camera.rgbd_camera import CameraIntrinsics, RGBDCameraInterface, RGBDFrame
from typing import Literal
import numpy as np



class KinectInterface(RGBDCameraInterface):
    def __init__(self, color_intrinsics: CameraIntrinsics, depth_intrinsics: CameraIntrinsics, T_color_depth:np.ndarray):
        """
        Initialize RGB-D interface.

        Args:
            color_intrinsics (CameraIntrinsics): Intrinsics for the color stream.
            depth_intrinsics (CameraIntrinsics): Intrinsics for the depth stream.
            T_color_depth (np.ndarray): (4, 4) homogeneous transform that maps points
                from the depth optical frame into the color optical frame.

        Conventions:
            - T_a_b maps coordinates expressed in frame b into frame a (p_a = T_a_b @ p_b).
            - Optical frames use the OpenCV convention: +Z forward, +X right, +Y down.
            - Color images are (H, W, 3) uint8 in RGB order.
            - Depth images are float32 meters.
        """
        super().__init__(color_intrinsics, depth_intrinsics, T_color_depth) 



    def start(self, resolution: tuple[int, int] = (640, 480), fps: int = 30,
        align: Literal["color", "depth"] = "color", device: str = None, serial: str = None):
        """
        Start the camera pipeline and begin streaming.

        Args:
            resolution (tuple[int, int]): (width, height) for enabled streams.
            fps (int): Target frame rate in frames per second.
            align ({"color", "depth", "none"}): Alignment behavior:
                - "color": depth is resampled into the color frame,
                - "depth": color is resampled into the depth frame,
            device (str): Device path/URI if needed by the backend.
            serial (str): Camera serial number when multiple devices are present.
        """
        ...



    def stop(self):
        """
        Stop streaming and release device resources.
        """
        ...


    def is_running(self) -> bool:
        """
        Check whether the camera is currently streaming.

        Returns:
            bool: True if streaming, False otherwise.
        """
        ...


    def latest(self) -> RGBDFrame:
        """
        Get the most recent RGB-D frame without blocking.

        Returns:
            RGBDFrame: The latest available frame.

        Raises:
            RuntimeError: If no frame has been captured yet.
        """
        ...


if __name__ == "__main__":
    import os

    cur_dir = os.path.dirname(__file__)
    # TODO:Update this config with the correct numbers
    config_path = os.path.join(cur_dir, "config", "kinect_config.yaml")

    kinect = KinectInterface.from_yaml(config_path)