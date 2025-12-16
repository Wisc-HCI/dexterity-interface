from __future__ import annotations

import logging
import time
from threading import Lock
from typing import Literal

import numpy as np

from sensor_interface.camera.rgbd_camera import (
    CameraIntrinsics,
    RGBDCameraInterface,
    RGBDFrame,
)

try:  # Local import to avoid requiring the SDK when not using Kinect
    _PYK4A_IMPORT_ERROR: Exception | None = None
    from pyk4a import (
        Config,
        FPS,
        ColorResolution,
        DepthMode,
        ImageFormat,
        PyK4A,
    )
    try:  # pyk4a <=1.4
        from pyk4a import PyK4AException
    except ImportError:  # pyk4a >=1.5 renamed to K4AException
        from pyk4a import K4AException as PyK4AException  # type: ignore

    import pyk4a.transformation as _k4a_transformation
    _TransformationClass = getattr(_k4a_transformation, "Transformation", None)
    _transform_depth_to_color = getattr(
        _k4a_transformation, "depth_image_to_color_camera", None
    )
    _transform_color_to_depth = getattr(
        _k4a_transformation, "color_image_to_depth_camera", None
    )
except Exception as exc:  # pragma: no cover - handled at runtime
    _PYK4A_IMPORT_ERROR = exc
    PyK4A = None  # type: ignore
    PyK4AException = Exception  # type: ignore
    _k4a_transformation = None
    _TransformationClass = None
    _transform_depth_to_color = None
    _transform_color_to_depth = None


_LOGGER = logging.getLogger(__name__)


class _TransformationAdapter:
    """
    Wraps pyk4a transformation API differences across versions.

    pyk4a <=1.4 exposes a Transformation class; >=1.5 exposes module-level
    functions that require the calibration and a thread_safe flag.
    """

    def __init__(self, calibration):
        self._calibration = calibration
        self._obj = _TransformationClass(calibration) if _TransformationClass else None

    def depth_to_color(self, depth: np.ndarray) -> np.ndarray | None:
        if self._obj is not None:
            return self._obj.depth_image_to_color_camera(depth)
        if _transform_depth_to_color is not None:
            return _transform_depth_to_color(
                depth, self._calibration, thread_safe=True
            )
        return None

    def color_to_depth(self, color: np.ndarray, depth: np.ndarray) -> np.ndarray | None:
        if self._obj is not None:
            return self._obj.color_image_to_depth_camera(color, depth)
        if _transform_color_to_depth is not None:
            return _transform_color_to_depth(
                color, depth, self._calibration, thread_safe=True
            )
        return None


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
        self._device: PyK4A | None = None
        self._transform: _TransformationAdapter | None = None
        self._latest_frame: RGBDFrame | None = None
        self._align: Literal["color", "depth"] = "color"
        self._lock = Lock()
        self._running = False



    def start(self, resolution: tuple[int, int] | None = None, fps: int = 30,
        align: Literal["color", "depth", "none"] = "color", device: int | None = None, serial: str | None = None):
        """
        Start the camera pipeline and begin streaming.

        Args:
            resolution (tuple[int, int] | None): (width, height) for enabled streams.
                Defaults to the calibrated color resolution from the YAML config.
            fps (int): Target frame rate in frames per second.
            align ({"color", "depth", "none"}): Alignment behavior:
                - "color": depth is resampled into the color frame,
                - "depth": color is resampled into the depth frame,
                - "none": frames are returned without additional alignment,
            device (int): Optional device index when multiple Kinect devices are present.
            serial (str): Not supported by PyK4A; use `device` index instead.
        """
        if PyK4A is None:
            raise ImportError(
                "pyk4a is required for Kinect streaming. Install Azure Kinect SDK and "
                "`pip install pyk4a`. Underlying import error: "
                f"{_PYK4A_IMPORT_ERROR}"
            ) from _PYK4A_IMPORT_ERROR

        if self._running:
            _LOGGER.debug("KinectInterface already running; ignoring start request.")
            return

        if align not in ("color", "depth", "none"):
            raise ValueError("align must be either 'color', 'depth', or 'none'")

        if resolution is None:
            resolution = (
                int(self.color_intrinsics.width),
                int(self.color_intrinsics.height),
            )

        color_res = self._map_color_resolution(resolution)
        depth_mode = self._select_depth_mode(resolution)
        fps_enum = self._map_fps(fps)

        config = Config(
            color_resolution=color_res,
            color_format=ImageFormat.COLOR_BGRA32,
            depth_mode=depth_mode,
            synchronized_images_only=True,
            camera_fps=fps_enum,
        )

        device_kwargs: dict = {}
        if device is not None:
            device_kwargs["device_id"] = int(device)
        if serial is not None:
            raise ValueError(
                "PyK4A does not support selecting a Kinect by serial number; "
                "pass a device index instead."
            )

        self._align = align
        self._device = PyK4A(config=config, **device_kwargs)
        self._device.start()
        self._transform = (
            _TransformationAdapter(self._device.calibration)
            if align in ("color", "depth")
            else None
        )
        self._running = True
        _LOGGER.info(
            "Started Kinect stream: color=%s depth_mode=%s fps=%s align=%s",
            color_res,
            depth_mode,
            fps_enum,
            align,
        )



    def stop(self):
        """
        Stop streaming and release device resources.
        """
        if self._device is not None:
            try:
                self._device.stop()
            finally:
                self._device = None

        self._transform = None
        self._running = False
        self._latest_frame = None


    def is_running(self) -> bool:
        """
        Check whether the camera is currently streaming.

        Returns:
            bool: True if streaming, False otherwise.
        """
        return self._running


    def latest(self) -> RGBDFrame:
        """
        Get the most recent RGB-D frame without blocking.

        Returns:
            RGBDFrame: The latest available frame.

        Raises:
            RuntimeError: If no frame has been captured yet.
        """
        if PyK4A is None:
            raise RuntimeError("pyk4a is not installed; cannot read Kinect frames.")

        if not self._running or self._device is None:
            raise RuntimeError("KinectInterface.start() must be called before reading.")

        try:
            # pyk4a<=1.4 uses positional timeout; 1.5 still supports positional only.
            capture = self._device.get_capture(0)
        except PyK4AException:
            capture = None

        if capture is None:
            with self._lock:
                if self._latest_frame is None:
                    raise RuntimeError("No frames captured yet.")
                return self._latest_frame

        color_bgra = capture.color
        depth_raw = capture.depth

        if (
            self._transform is not None
            and depth_raw is not None
            and color_bgra is not None
        ):
            if self._align == "color":
                depth_raw = self._transform.depth_to_color(depth_raw)
            elif self._align == "depth":
                color_bgra = self._transform.color_to_depth(color_bgra, depth_raw)

        color_rgb = None
        if color_bgra is not None:
            # BGRA -> RGB, drop alpha channel
            color_rgb = color_bgra[..., :3][:, :, ::-1].copy()

        depth_m = None
        if depth_raw is not None:
            depth_m = depth_raw.astype(np.float32) / 1000.0

        frame_id = (
            "kinect_depth_optical_frame"
            if self._align == "depth"
            else "kinect_color_optical_frame"
        )
        frame = RGBDFrame(
            color=color_rgb,
            depth=depth_m,
            timestamp=time.time(),
            frame_id=frame_id,
        )
        with self._lock:
            self._latest_frame = frame
        return frame

    ########################### Helpers ###########################

    def _map_color_resolution(self, resolution: tuple[int, int]) -> ColorResolution:
        """
        Map a requested (width, height) to the closest supported Kinect color resolution.

        Args:
            resolution (tuple[int, int]): Desired (width, height) for the color stream.

        Returns:
            ColorResolution: The matching Kinect enum. Falls back to RES_720P if the
            requested size is unsupported.
        """
        mapping = {
            (1280, 720): ColorResolution.RES_720P,
            (1920, 1080): ColorResolution.RES_1080P,
            (2560, 1440): ColorResolution.RES_1440P,
            (2048, 1536): ColorResolution.RES_1536P,
            (3840, 2160): ColorResolution.RES_2160P,
            (4096, 3072): ColorResolution.RES_3072P,
        }
        if resolution in mapping:
            return mapping[resolution]

        _LOGGER.warning(
            "Requested resolution %s not supported; defaulting to 1280x720", resolution
        )
        return ColorResolution.RES_720P

    @staticmethod
    def _map_fps(fps: int) -> FPS:
        """
        Convert a requested integer FPS into the nearest Kinect-supported enum.

        Args:
            fps (int): Requested frames per second.

        Returns:
            FPS: The closest supported frame rate (5, 15, or 30).
        """
        if fps <= 5:
            return FPS.FPS_5
        if fps <= 15:
            return FPS.FPS_15
        return FPS.FPS_30

    @staticmethod
    def _select_depth_mode(resolution: tuple[int, int]) -> DepthMode:
        """
        Choose a depth mode that roughly matches the requested color resolution.

        Args:
            resolution (tuple[int, int]): Desired color (width, height).

        Returns:
            DepthMode: Wide FOV for resolutions >=1920, wide binned for mid-range,
            otherwise narrow FOV unbinned (640x576 equivalent).
        """
        if resolution[0] >= 1920:
            return DepthMode.WFOV_UNBINNED
        if resolution[0] >= 1024:
            return DepthMode.WFOV_2X2BINNED
        return DepthMode.NFOV_UNBINNED


if __name__ == "__main__":
    import os

    cur_dir = os.path.dirname(__file__)
    config_path = os.path.join(cur_dir, "config", "kinect_config.yaml")

    kinect = KinectInterface.from_yaml(config_path)
