from abc import abstractmethod
from dataclasses import dataclass
from typing import Literal
import yaml
import numpy as np


@dataclass
class CameraIntrinsics:
    """
    Pin-hole camera intrinsics for a single stream. 
    Uses Brown distortion model.

    Args:
        width (int): Image width in pixels.
        height (int): Image height in pixels.
        fx (float): Focal length in pixels (x-axis).
        fy (float): Focal length in pixels (y-axis).
        cx (float): Principal point x-coordinate in pixels.
        cy (float): Principal point y-coordinate in pixels.
        distortion (np.ndarray): (5,) Distortion parameters for Brown 
            distortion model [k1, k2, p1, p2, k3]

        TODO: Not sure if we need distortion model (and which one we need)
    """
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: np.ndarray | None = None

    @classmethod
    def from_dict(cls, data: dict):
        return cls(
            width=data["width"],
            height=data["height"],
            fx=data["fx"],
            fy=data["fy"],
            cx=data["cx"],
            cy=data["cy"],
            distortion=np.array(data.get("distortion", []), dtype=float)
        )


@dataclass
class RGBDFrame:
    """
    Synchronized RGB-D frame with metadata.

    Args:
        color (np.ndarray): (H, W, 3) uint8 RGB image in sRGB order, or None if color is disabled.
        depth (np.ndarray): (H, W) float32 depth in m
        timestamp (float): Capture time in seconds (monotonic or device clock; document in subclass).
        frame_id (str): Camera optical frame name for this capture.
    """
    color: np.ndarray | None
    depth: np.ndarray | None
    timestamp: float
    frame_id: str


class RGBDCameraInterface:
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
        self.color_intrinsics = color_intrinsics
        self.depth_intrinsics = depth_intrinsics
        self.T_color_depth = T_color_depth

    @classmethod
    def from_yaml(cls, filename: str):
        """
        Construct an RGBDCameraInterface from a YAML configuration file.

        The YAML file should define the following keys:
            - color_intrinsics: dict with same key/values as CameraIntrinsics
            - depth_intrinsics: dict with same key/values as CameraIntrinsics
            - T_color_depth: 4x4 list (row-major) representing the homogeneous transform
              that maps points from the depth optical frame into the color optical frame

        Args:
            filename (str): Path to the YAML configuration file.

        Returns:
            RGBDCameraInterface: An initialized RGB-D camera interface containing
            validated color and depth intrinsics along with the transform between them.
        """
        with open(filename, "r") as f:
            config = yaml.safe_load(f)

        color_intr = CameraIntrinsics.from_dict(config["color_intrinsics"])
        depth_intr = CameraIntrinsics.from_dict(config["depth_intrinsics"])
        T_color_depth = np.array(config["T_color_depth"], dtype=float)

        # Optional frame ID for this camera; used in RGBDFrame.frame_id
        frame_id = config.get("frame_id", "camera_optical_frame")

        # Construct the concrete camera (e.g., RealsenseInterface)
        camera = cls(color_intr, depth_intr, T_color_depth)

        # Store frame_id
        setattr(camera, "_frame_id", frame_id)

        return camera



    @abstractmethod
    def start(self, resolution: tuple[int, int] = (640, 480), fps: int = 30,
        align: Literal["color", "depth"] = "color", serial: str = None):
        """
        Start the camera pipeline and begin streaming.

        Args:
            resolution (tuple[int, int]): (width, height) for enabled streams.
            fps (int): Target frame rate in frames per second.
            align ({"color", "depth", "none"}): Alignment behavior:
                - "color": depth is resampled into the color frame,
                - "depth": color is resampled into the depth frame,
            serial (str): Camera serial number when multiple devices are present.
        """
        ...


    @abstractmethod
    def stop(self):
        """
        Stop streaming and release device resources.
        """
        ...


    @abstractmethod
    def is_running(self) -> bool:
        """
        Check whether the camera is currently streaming.

        Returns:
            bool: True if streaming, False otherwise.
        """
        ...

    @abstractmethod
    def latest(self) -> RGBDFrame:
        """
        Get the most recent RGB-D frame without blocking.

        Returns:
            RGBDFrame: The latest available frame.

        Raises:
            RuntimeError: If no frame has been captured yet.
        """
        ...


    ################################# Utils #################################

    def depth_to_pointcloud(self, depth: np.ndarray) -> np.ndarray:
        """
        Convert a depth image into a point cloud in the depth optical frame
        using the current depth intrinsics.

        Args:
            depth (np.ndarray): (H, W) float32 depth in meters.

        Returns:
            np.ndarray: (N, 3) float32 XYZ points in meters, where N = H*W or a
           
                filtered subset if the implementation skips invalid pixels.
        """

        # TODO: Implement this since this is camera independent
        ...
