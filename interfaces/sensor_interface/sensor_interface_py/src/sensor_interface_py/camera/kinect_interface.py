from interfaces.sensor_interface.sensor_interface_py.src.sensor_interface_py.camera.rgbd_camera import CameraIntrinsics, RGBDCameraInterface, RGBDFrame
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
   
    # MADE UP FOR TESTING
    color_intrinsics = CameraIntrinsics(
        width=640, height=480,
        fx=525.0, fy=525.0,
        cx=319.5, cy=239.5,                      # (width-1)/2, (height-1)/2
        distortion=np.array([0.10, -0.15, 0.0, 0.0, 0.0], dtype=np.float32),
        model=""
    )
    depth_intrinsics = CameraIntrinsics(
        width=640, height=480,
        fx=575.0, fy=575.0,
        cx=319.5, cy=239.5,
        distortion=np.zeros(5, dtype=np.float32),
        model=""
    )
    T_color_depth = np.eye(4, dtype=np.float32)
    T_color_depth = np.array([
        [ 0.99985,  0.0,     0.01745,  0.020],
        [ 0.0,      1.0,     0.0,     -0.010],
        [-0.01745,  0.0,     0.99985,  0.000],
        [ 0.0,      0.0,     0.0,      1.000],
    ], dtype=np.float32)
    T_color_depth[:3,  3] = np.array([0.020, -0.010, 0.000], dtype=np.float32)  # meters
        
    kinect = KinectInterface(color_intrinsics, depth_intrinsics, T_color_depth)