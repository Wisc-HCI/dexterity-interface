"""
RealSense RGB-D camera interface using pyrealsense2.

This module implements the RGBDCameraInterface for Intel RealSense devices.
It supports streaming color + depth, optional alignment, and a non-blocking
latest() call via a background capture thread.

Conventions:
- Color images: (H, W, 3) uint8 in RGB order.
- Depth images: (H, W) float32 in meters.
"""

from __future__ import annotations

from sensor_interface.camera.rgbd_camera import CameraIntrinsics, RGBDCameraInterface, RGBDFrame
from typing import Literal, Optional
import numpy as np
import threading
import time


try:
    import pyrealsense2 as rs
except ImportError as e:
    raise ImportError(
        "pyrealsense2 is required for RealsenseInterface. "
        "Install it (macOS): pip install pyrealsense2-macosx"
    ) from e

class RealsenseInterface(RGBDCameraInterface):
    def __init__(
        self,
        color_intrinsics: CameraIntrinsics,
        depth_intrinsics: CameraIntrinsics,
        T_color_depth: np.ndarray,
        frame_id: str = "camera_optical_frame",
    ):
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
        super().__init__(color_intrinsics, depth_intrinsics, T_color_depth, frame_id=frame_id)

        self._pipeline: Optional[rs.pipeline] = None
        self._config: Optional[rs.config] = None
        self._aligner: Optional[rs.align] = None
        self._running: bool = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        self._latest_frame: Optional[RGBDFrame] = None
        self._depth_scale: float = 1.0


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

        Raises:
            RuntimeError: If already running or if RealSense fails to start.
        """

        if self._running:
            raise RuntimeError("RealsenseInterface is already running.")

        width, height = resolution

        self._pipeline = rs.pipeline()
        self._config = rs.config()

        if serial is not None:
            self._config.enable_device(serial)

        # Enable streams - Expect width/height/fps
        self._config.enable_stream(
            rs.stream.color, width, height, rs.format.rgb8, fps
        )
        self._config.enable_stream(
            rs.stream.depth, width, height, rs.format.z16, fps
        )

        try:
            profile = self._pipeline.start(self._config)
        except Exception as e:
            self._pipeline = None
            self._config = None
            raise RuntimeError(f"Failed to start RealSense pipeline: {e}") from e

        # Convert depth to meters
        try:
            depth_sensor = profile.get_device().first_depth_sensor()
            self._depth_scale = float(depth_sensor.get_depth_scale())
        except Exception:
            self._depth_scale = 1.0

        if align == "color":
            self._aligner = rs.align(rs.stream.color)
        elif align == "depth":
            self._aligner = rs.align(rs.stream.depth)
        else:
            self._aligner = None

        # Stabilization
        for _ in range(5):
            try:
                _ = self._pipeline.wait_for_frames()
            except Exception:
                pass

        self._running = True
        self._thread = threading.Thread(
            target=self._capture_loop, name="realsense_capture_loop", daemon=True
        )
        self._thread.start()


    def stop(self):
        """
        Stop streaming and release device resources.
        """
        print("[INFO] Stopping camera loop.")
        if not self._running:
            return

        self._running = False

        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass

        self._pipeline = None
        self._config = None
        self._aligner = None

        with self._lock:
            self._latest_frame = None


    def is_running(self) -> bool:
        """
        Check whether the camera is currently streaming.

        Returns:
            bool: True if streaming, False otherwise.
        """
        return bool(self._running)


    def latest(self) -> RGBDFrame:
        """
        Get the most recent RGB-D frame without blocking.

        Returns:
            RGBDFrame: The latest available frame.

        Raises:
            RuntimeError: If no frame has been captured yet.
        """
        with self._lock:
            if self._latest_frame is None:
                raise RuntimeError("No frame has been captured yet.")
            return self._latest_frame


    # Private helper
    def _capture_loop(self):
        """
        Background thread loop that continuously reads RealSense frames,
        applies alignment if enabled, converts them to numpy, and stores
        the latest RGBDFrame.

        This function should not be called directly by users.
        """

        assert self._pipeline is not None

        while self._running:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
            except Exception:
                continue

            if self._aligner is not None:
                try:
                    frames = self._aligner.process(frames)
                except Exception:
                    continue

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            # Convert to numpy
            color = np.asanyarray(color_frame.get_data())  # RGB uint8
            depth_raw = np.asanyarray(depth_frame.get_data()).astype(np.float32)
            depth = depth_raw * self._depth_scale  # meters

            timestamp = float(color_frame.get_timestamp()) / 1000.0  # ms to s

            rgbd_frame = RGBDFrame(
                color=color,
                depth=depth,
                timestamp=timestamp,
                frame_id=self.frame_id,
            )

            with self._lock:
                self._latest_frame = rgbd_frame

            # Small sleep to avoid busy-waiting
            time.sleep(0.001)


if __name__ == "__main__":
    import os

    cur_dir = os.path.dirname(__file__)
    # Update this config with the correct numbers (done)
    config_path = os.path.join(cur_dir, "config", "realsense_config.yaml")

    camera = RealsenseInterface.from_yaml(config_path)
    camera.start()
    print("RealSense started. Waiting for first frame...")

    # Wait for the first valid frame instead of failing immediately
    frame = None
    start_time = time.time()
    while frame is None:
        try:
            frame = camera.latest()
        except RuntimeError:
            frame = None

        if time.time() - start_time > 3.0:
            camera.stop()
            raise RuntimeError("Camera failed to provide frames after 3 seconds.")

        time.sleep(0.05)

    print("RealSense streaming. Press Ctrl+C to stop.")
    try:
        while True:
            try:
                frame = camera.latest()
            except RuntimeError:
                continue

            print(
                f"Got frame: color {frame.color.shape if frame.color is not None else None}, "
                f"depth {frame.depth.shape if frame.depth is not None else None}"
            )
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
        print("RealSense stopped.")
        
