from __future__ import annotations

import threading
import time
from collections import deque
from typing import Literal, Optional, Tuple

import numpy as np

from sensor_interface.camera.rgbd_camera import CameraIntrinsics, RGBDCameraInterface, RGBDFrame

try:
    # Using pyk4a (Azure Kinect SDK Python bindings)
    import pyk4a
    from pyk4a import PyK4A, Config, ColorResolution, DepthMode, ImageFormat, FPS, transformation
    _HAVE_K4A = True
except Exception:
    _HAVE_K4A = False


class KinectInterface(RGBDCameraInterface):
    """
    Concrete RGB‑D camera interface for the Azure Kinect.

    Responsibilities:
        - Configure and start/stop the device
        - Acquire synchronized color/depth captures in a background thread
        - Convert device-native formats (BGRA, mm) to standard ones (RGB uint8, m float32)
        - Optionally align depth to color or color to depth

    Invariants:
        - `self._running` is True iff the capture thread is alive and the device is started.
        - The queue `self._frame_q` contains at most one frame (latest only).
    """

    def __init__(self, color_intrinsics: CameraIntrinsics, depth_intrinsics: CameraIntrinsics, T_color_depth: np.ndarray) -> None:
        """
        Initialize an Azure Kinect RGB‑D camera interface.

        Args:
            color_intrinsics (CameraIntrinsics): Intrinsics for the color stream.
            depth_intrinsics (CameraIntrinsics): Intrinsics for the depth stream.
            T_color_depth (np.ndarray): (4, 4) homogeneous transform mapping points
                from the depth optical frame into the color optical frame.

        Notes:
            Optical frames follow the OpenCV convention: +Z forward, +X right, +Y down.
            Color images are (H, W, 3) uint8 in RGB order. Depth images are (H, W) float32 in meters.
        """
        super().__init__(color_intrinsics, depth_intrinsics, T_color_depth)

        self._device: Optional[PyK4A] = None
        self._transformation: Optional[transformation] = None
        self._align: Literal["color", "depth"] = "color"
        self._running: bool = False
        self._thread: Optional[threading.Thread] = None
        self._frame_q: deque[RGBDFrame] = deque(maxlen=1)
        self._serial: Optional[str] = None

    def start(
        self,
        resolution: tuple[int, int] = (640, 480),
        fps: int = 30,
        align: Literal["color", "depth"] = "color",
        device: Optional[str] = None,
        serial: Optional[str] = None,
    ) -> None:
        """
        Start the Azure Kinect device and begin streaming synchronized RGB‑D frames.

        Args:
            resolution (tuple[int, int]): (W, H) target resolution used to pick closest
                color and depth modes.
            fps (int): Frame rate in FPS. Supported set is {5, 15, 30}. Nearest value is used.
            align ({"color", "depth"}): Alignment behavior.
                - "color": resample depth into the color frame; outputs roughly match color size.
                - "depth": resample color into the depth frame; outputs roughly match depth size.
            device (Optional[str]): Device path/URI (not used; kept for API parity).
            serial (Optional[str]): Device serial to select among multiple connected devices.

        Raises:
            ImportError: If `pyk4a`/runtime is not available.
        """
        if not _HAVE_K4A:
            raise ImportError(
                "pyk4a is not installed. Install with `pip install pyk4a` and set up Azure Kinect SDK/runtime."
            )

        if fps not in (5, 15, 30):
            fps = min((5, 15, 30), key=lambda x: abs(x - fps))

        self._align = align
        self._serial = serial

        color_res_enum = _pick_color_resolution(resolution)
        depth_mode_enum = _pick_depth_mode(resolution)

        cfg = Config(
            color_resolution=color_res_enum,
            color_format=ImageFormat.COLOR_BGRA32,
            depth_mode=depth_mode_enum,
            camera_fps={5: FPS.FPS_5, 15: FPS.FPS_15, 30: FPS.FPS_30}[fps],
            synchronized_images_only=True,
        )

        self._device = PyK4A(cfg, device_id=_resolve_device_index(serial))
        self._device.start()
        self._transformation = transformation(self._device.calibration)

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, name="KinectInterfaceCapture", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """
        Stop streaming and release device resources.
        """
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.5)
        self._thread = None

        if self._device is not None:
            try:
                self._device.stop()
            except Exception:
                pass
        self._device = None
        self._transformation = None

    def is_running(self) -> bool:
        """
        Check whether the camera pipeline is currently streaming.

        Returns:
            bool: True if streaming; False otherwise.
        """
        return bool(self._running and self._device is not None)

    def latest(self) -> RGBDFrame:
        """
        Get the most recently captured RGB‑D frame without blocking.

        Returns:
            RGBDFrame: Latest frame with attributes:
                - color (np.ndarray | None): (H, W, 3) uint8 RGB or None.
                - depth (np.ndarray | None): (H, W) float32 meters or None.
                - timestamp_ns (int): Nanosecond capture timestamp if supported by RGBDFrame type.

        Raises:
            RuntimeError: If no frame has been captured yet.
        """
        if not self._frame_q:
            raise RuntimeError("No frame captured yet. Is the device running?")
        return self._frame_q[0]

    def _capture_loop(self) -> None:
        """
        Internal thread loop that:
            1) Pulls captures from the device
            2) Converts color BGRA→RGB and depth mm→meters
            3) Applies alignment (depth→color or color→depth) as requested
            4) Enqueues the latest frame (capacity-1 queue)
        """
        assert self._device is not None
        warmup = 10
        while self._running:
            try:
                capture = self._device.get_capture(timeout_ms=2000)
            except Exception:
                if not self._running:
                    break
                continue

            if capture is None:
                continue

            color = capture.color
            depth = capture.depth

            if color is not None and color.ndim == 3 and color.shape[2] == 4:
                color_bgr = color[:, :, :3].copy()
                color = color_bgr[:, :, ::-1]

            depth_m: Optional[np.ndarray] = None
            if depth is not None and depth.dtype == np.uint16:
                depth_m = depth.astype(np.float32) / 1000.0

            color_aligned = color
            depth_aligned = depth_m

            if self._transformation is not None and color is not None and depth is not None:
                if self._align == "color":
                    depth_to_color_mm = self._transformation.depth_image_to_color_camera(depth)
                    depth_aligned = depth_to_color_mm.astype(np.float32) / 1000.0
                    color_aligned = color
                elif self._align == "depth":
                    color_to_depth_bgra = self._transformation.color_image_to_depth_camera(depth, capture.color)
                    if color_to_depth_bgra is not None:
                        color_aligned = color_to_depth_bgra[:, :, :3][:, :, ::-1].copy()
                    depth_aligned = depth_m

            ts_ns = _now_ns()

            try:
                frame = RGBDFrame(
                    color=color_aligned if color_aligned is not None else None,
                    depth=depth_aligned if depth_aligned is not None else None,
                    timestamp_ns=ts_ns,
                    serial=self._serial,
                )
            except TypeError:
                frame = RGBDFrame(
                    color_aligned if color_aligned is not None else None,
                    depth_aligned if depth_aligned is not None else None,
                    ts_ns,
                )

            self._frame_q.append(frame)

            if warmup > 0:
                warmup -= 1

            if not self._running:
                break


def _now_ns() -> int:
    """
    Return a monotonic timestamp in nanoseconds.

    Returns:
        int: Monotonic time in nanoseconds based on `time.time_ns()` when available,
        otherwise `int(time.time() * 1e9)`.
    """
    try:
        return time.time_ns()
    except AttributeError:
        return int(time.time() * 1e9)


def _resolve_device_index(serial: Optional[str]) -> int:
    """
    Map a device serial number to a `PyK4A` device index.

    Args:
        serial (Optional[str]): Serial number string for the target device, or None to
            select the default (first) device.

    Returns:
        int: Device index in [0, N-1]. Currently always returns 0.
    """
    return 0


def _pick_color_resolution(res: Tuple[int, int]) -> ColorResolution:
    """
    Choose the nearest Azure Kinect `ColorResolution` enum for a requested size.

    Args:
        res (tuple[int, int]): (W, H) requested resolution.

    Returns:
        ColorResolution: Closest matching color resolution enum among the supported set.
    """
    w, h = res
    candidates = {
        (1280, 720): ColorResolution.RES_720P,
        (1920, 1080): ColorResolution.RES_1080P,
        (2560, 1440): ColorResolution.RES_1440P,
        (2048, 1536): ColorResolution.RES_1536P,
        (3840, 2160): ColorResolution.RES_2160P,
        (4096, 3072): ColorResolution.RES_3072P,
    }
    key = min(candidates.keys(), key=lambda wh: abs(wh[0] - w) + abs(wh[1] - h))
    return candidates[key]


def _pick_depth_mode(res: Tuple[int, int]) -> DepthMode:
    """
    Choose an Azure Kinect `DepthMode` that roughly matches a requested size.

    Args:
        res (tuple[int, int]): (W, H) requested resolution.

    Returns:
        DepthMode: One of {NFOV_2X2BINNED, WFOV_2X2BINNED, NFOV_UNBINNED, WFOV_UNBINNED}.
    """
    w, h = res
    if max(w, h) >= 1000:
        return DepthMode.WFOV_UNBINNED
    if max(w, h) >= 600:
        return DepthMode.NFOV_UNBINNED
    if max(w, h) >= 500:
        return DepthMode.WFOV_2X2BINNED
    return DepthMode.NFOV_2X2BINNED


if __name__ == "__main__":
    import os

    cur_dir = os.path.dirname(__file__)
    config_path = os.path.join(cur_dir, "config", "kinect_config.yaml")

    kinect = KinectInterface.from_yaml(config_path)
    kinect.start(resolution=(1280, 720), fps=30, align="color")

    try:
        for _ in range(30):
            if kinect.is_running():
                frame = kinect.latest()
                c = getattr(frame, "color", None)
                d = getattr(frame, "depth", None)
                print(
                    "color:", None if c is None else c.shape,
                    "depth:", None if d is None else d.shape,
                )
            time.sleep(0.1)
    finally:
        kinect.stop()
