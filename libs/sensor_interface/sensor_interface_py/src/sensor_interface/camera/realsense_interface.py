from __future__ import annotations

import threading
import time
from collections import deque
from typing import Literal, Optional, Tuple

import numpy as np
from sensor_interface.camera.rgbd_camera import CameraIntrinsics, RGBDCameraInterface, RGBDFrame

try:
    import pyrealsense2 as rs
    _HAVE_RS = True
except Exception:
    _HAVE_RS = False



class RealsenseInterface(RGBDCameraInterface):
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

        self._pipeline: Optional[rs.pipeline] = None
        self._align_mode: Literal["color", "depth", "none"] = "color"
        self._align_obj: Optional[rs.align] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._frame_q: deque[RGBDFrame] = deque(maxlen=1)
        self._serial: Optional[str] = None
        self._depth_scale: float = 0.001


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

        if not _HAVE_RS:
            raise ImportError("pyrealsense2 is not installed. Install with `pip install pyrealsense2`.")

        self._align_mode = align
        self._serial = serial

        w, h = resolution

        pipeline = rs.pipeline()
        cfg = rs.config()
        if serial:
            cfg.enable_device(serial)

        # Try to enable both streams at the requested resolution. If the combo
        # isn't supported, the start() call will raise; that’s fine—surface the error.
        cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        # Use RGB to avoid later channel swapping; many examples use BGR, but RGB is simpler here.
        cfg.enable_stream(rs.stream.color, w, h, rs.format.rgb8, fps)

        # Start pipeline
        profile = pipeline.start(cfg)

        # Read depth scale (units → meters)
        depth_sensor = profile.get_device().first_depth_sensor()
        self._depth_scale = float(depth_sensor.get_depth_scale())  # typically 0.001

        # Prepare align object
        if self._align_mode == "color":
            self._align_obj = rs.align(rs.stream.color)
        elif self._align_mode == "depth":
            self._align_obj = rs.align(rs.stream.depth)
        else:
            self._align_obj = None

        self._pipeline = pipeline
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, name="RealSenseCapture", daemon=True)
        self._thread.start()

        ...



    def stop(self):
        """
        Stop streaming and release device resources.
        """

        """Stop streaming and release device resources."""

        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.5)
        self._thread = None

        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
        self._pipeline = None
        self._align_obj = None

        ...


    def is_running(self) -> bool:
        """
        Check whether the camera is currently streaming.

        Returns:
            bool: True if streaming, False otherwise.
        """

        return bool(self._running and self._pipeline is not None)
        ...


    def latest(self) -> RGBDFrame:
        """
        Get the most recent RGB-D frame without blocking.

        Returns:
            RGBDFrame: The latest available frame.

        Raises:
            RuntimeError: If no frame has been captured yet.
        """

        if not self._frame_q:
            raise RuntimeError("No frame captured yet. Is the device running?")
        return self._frame_q[0]
        ...


    def _capture_loop(self):
        assert self._pipeline is not None
        warmup = 10

        while self._running:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=2000)
            except Exception:
                if not self._running:
                    break
                continue

            if frames is None:
                continue

            if self._align_obj is not None:
                frames = self._align_obj.process(frames)

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert to numpy
            color = np.asanyarray(color_frame.get_data())  
            depth_raw = np.asanyarray(depth_frame.get_data()) 

            # Scale depth to meters (float32)
            depth_m = (depth_raw.astype(np.float32) * self._depth_scale)  

            # Note: If align == "color", color and depth share color resolution.
            # If align == "depth", they share depth resolution. If "none", they remain native.

            ts_ns = _now_ns()
            try:
                frame = RGBDFrame(color=color, depth=depth_m, timestamp_ns=ts_ns, serial=self._serial)
            except TypeError:
                frame = RGBDFrame(color, depth_m, ts_ns)

            self._frame_q.append(frame)

            if warmup > 0:
                warmup -= 1

            if not self._running:
                break


def _now_ns() -> int:
        try:
            return time.time_ns()
        except AttributeError:
            return int(time.time() * 1e9)


if __name__ == "__main__":
    import os
    cur_dir = os.path.dirname(__file__)
    config_path = os.path.join(cur_dir, "config", "realsense_config.yaml")
    rs_cam = RealsenseInterface.from_yaml(config_path)
    rs_cam.start(resolution=(640, 480), fps=30, align="color")
    try:
        for _ in range(30):
            if rs_cam.is_running():
                f = rs_cam.latest()
                c = getattr(f, "color", None)
                d = getattr(f, "depth", None)
                print("color:", None if c is None else c.shape, "depth:", None if d is None else d.shape)
            time.sleep(0.1)
    finally:
        rs_cam.stop()