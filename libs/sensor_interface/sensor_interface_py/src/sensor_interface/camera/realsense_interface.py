from __future__ import annotations

import threading
import time
from typing import Optional, Literal, Tuple

import numpy as np

from sensor_interface.camera.rgbd_camera import (
    CameraIntrinsics,
    RGBDCameraInterface,
    RGBDFrame,
)

# Optional dependency: Intel RealSense
try:
    import pyrealsense2 as rs
    _HAVE_RS = True
except Exception as e:
    print("pyrealsense2 import failed:", repr(e))
    _HAVE_RS = False


def _now_ns() -> int:
    """
    Get a monotonic-ish timestamp in nanoseconds.

    Args:
        (no args)

    Returns:
        (int): Current time in nanoseconds.
    """
    try:
        return time.time_ns()
    except AttributeError:
        return int(time.time() * 1e9)


class RealsenseInterface(RGBDCameraInterface):
    def __init__(
        self,
        color_intrinsics: CameraIntrinsics,
        depth_intrinsics: CameraIntrinsics,
        T_color_depth: np.ndarray,
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
        super().__init__(color_intrinsics, depth_intrinsics, T_color_depth)

        # Runtime state (initialized when start() is called)
        self._pipeline: Optional["rs.pipeline"] = None
        self._config: Optional["rs.config"] = None
        self._align_mode: Literal["color", "depth"] = "color"
        self._align: Optional["rs.align"] = None
        self._depth_scale_m: float = 0.001  # meters per depth unit (default)
        self._serial_used: Optional[str] = None

        self._running: bool = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._latest: Optional[RGBDFrame] = None

    def start(
        self,
        resolution: Tuple[int, int] = (640, 480),
        fps: int = 30,
        align: Literal["color", "depth"] = "color",
        device: Optional[str] = None,
        serial: Optional[str] = None,
    ):
        """
        Start the camera pipeline and begin streaming.

        Args:
            resolution (tuple[int, int]): (width, height) for enabled streams.
            fps (int): Target frame rate in frames per second.
            align (Literal["color","depth"]): Alignment behavior:
                - "color": depth is resampled into the color frame.
                - "depth": color is resampled into the depth frame.
            device (Optional[str]): Unused for librealsense; kept for API parity.
            serial (Optional[str]): Camera serial number when multiple devices are present.

        Returns:
            (None)

        Raises:
            ImportError: If pyrealsense2 is unavailable.
            RuntimeError: If already running or pipeline fails to start.
        """
        if not _HAVE_RS:
            raise ImportError(
                "pyrealsense2 is not available. Install Intel RealSense SDK and `pip install pyrealsense2`."
            )
        if self._running:
            # Idempotent; do nothing if already running.
            return

        width, height = resolution
        self._align_mode = align

        # Configure pipeline
        self._pipeline = rs.pipeline()
        self._config = rs.config()

        # Serial selection (if provided)
        if serial:
            self._config.enable_device(serial)
            self._serial_used = serial
        else:
            self._serial_used = None

        # Enable color (RGB8) and depth (Z16)
        self._config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
        self._config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

        # Start pipeline
        try:
            profile: rs.pipeline_profile = self._pipeline.start(self._config)
        except Exception as e:
            # Reset partially constructed state
            self._pipeline = None
            self._config = None
            raise RuntimeError(f"Failed to start RealSense pipeline: {e!r}")

        # Resolve actual serial used
        if self._serial_used is None:
            try:
                dev = profile.get_device()
                self._serial_used = dev.get_info(rs.camera_info.serial_number)
            except Exception:
                self._serial_used = "UNKNOWN"

        # Query depth scale (meters per unit)
        try:
            dev = profile.get_device()
            depth_sensor = next((s for s in dev.sensors if s.is_depth_sensor()), None)
            if depth_sensor is not None:
                self._depth_scale_m = float(depth_sensor.get_depth_scale())
            else:
                self._depth_scale_m = 0.001
        except Exception:
            self._depth_scale_m = 0.001

        # Prepare alignment object
        if self._align_mode == "color":
            self._align = rs.align(rs.stream.color)
        elif self._align_mode == "depth":
            self._align = rs.align(rs.stream.depth)
        else:
            # Only "color" or "depth" are supported in this interface
            self._align = rs.align(rs.stream.color)

        # Start background capture
        self._running = True
        self._thread = threading.Thread(
            target=self._capture_loop, name="RealsenseCaptureLoop", daemon=True
        )
        self._thread.start()

    def stop(self):
        """
        Stop streaming and release device resources.

        Args:
            (no args)

        Returns:
            (None)
        """
        if not self._running:
            return

        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

        # Stop and clear pipeline
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            self._pipeline = None
        self._config = None
        self._align = None

        with self._lock:
            self._latest = None

    def is_running(self) -> bool:
        """
        Check whether the camera is currently streaming.

        Args:
            (no args)

        Returns:
            (bool): True if streaming, False otherwise.
        """
        return self._running

    def latest(self) -> RGBDFrame:
        """
        Get the most recent RGB-D frame without blocking.

        Args:
            (no args)

        Returns:
            (RGBDFrame): The latest available frame with fields:
                - color: (H, W, 3) uint8 RGB
                - depth: (H, W) float32 meters
                - timestamp_ns (if supported by your RGBDFrame)
                - serial (if supported by your RGBDFrame)

        Raises:
            RuntimeError: If no frame has been captured yet.
        """
        with self._lock:
            if self._latest is None:
                raise RuntimeError("No frame captured yet. Did you call start() and wait briefly?")
            return self._latest

    # ---------------- Internal helpers ----------------

    def _capture_loop(self):
        """
        Internal background loop that pulls frames from the device, performs
        optional alignment, converts depth to meters, and stores the latest frame.

        Args:
            (no args)

        Returns:
            (None)
        """
        assert self._pipeline is not None
        wait_ms = 5000

        while self._running:
            try:
                frames: "rs.frameset" = self._pipeline.wait_for_frames(timeout_ms=wait_ms)
            except Exception:
                # USB hiccup or timeout; continue if still running
                if not self._running:
                    break
                continue

            if self._align is not None:
                try:
                    frames = self._align.process(frames)
                except Exception:
                    # If alignment fails transiently, continue
                    pass

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Fetch numpy arrays
            color_rgb = np.asanyarray(color_frame.get_data())  # (H, W, 3) RGB8
            depth_u16 = np.asanyarray(depth_frame.get_data())  # (H, W) uint16

            # Convert depth to meters
            depth_m = depth_u16.astype(np.float32) * self._depth_scale_m

            # Timestamps (ms -> ns if available)
            try:
                ts_ms = depth_frame.get_timestamp()
                ts_ns = int(float(ts_ms) * 1e6)
            except Exception:
                ts_ns = _now_ns()

            # Package RGBDFrame; be tolerant of constructor signature differences
            try:
                frame = RGBDFrame(
                    color=color_rgb,
                    depth=depth_m,
                    timestamp_ns=ts_ns,
                    serial=self._serial_used,
                )
            except TypeError:
                frame = RGBDFrame(color_rgb, depth_m, ts_ns)

            with self._lock:
                self._latest = frame


if __name__ == "__main__":
    import os

    cur_dir = os.path.dirname(__file__)
    # TODO: Update this config with the correct numbers if using from_yaml
    config_path = os.path.join(cur_dir, "config", "realsense_config.yaml")

    # Example usage: relies on RGBDCameraInterface.from_yaml being defined in your base class.
    cam = RealsenseInterface.from_yaml(config_path)
    cam.start(resolution=(640, 480), fps=30, align="color")
    try:
        # Warm up a bit and print a few shapes
        for _ in range(30):
            if cam.is_running():
                f = cam.latest()
                c = getattr(f, "color", None)
                d = getattr(f, "depth", None)
                print("color:", None if c is None else c.shape, "depth:", None if d is None else d.shape)
            time.sleep(0.1)
    finally:
        cam.stop()
