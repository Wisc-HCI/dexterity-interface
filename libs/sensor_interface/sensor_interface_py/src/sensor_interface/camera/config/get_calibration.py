"""
Print Kinect calibration (intrinsics/extrinsics) in a YAML-friendly dict.

Usage (from repo root):
    python libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/get_calibration.py

Optional flags:
    --device <index>   # pick device index if multiple are connected
    --serial <serial>  # pick by serial number
"""

from __future__ import annotations

import argparse
import sys
from pprint import pprint

import numpy as np


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dump Azure Kinect calibration.")
    parser.add_argument("--device", type=int, default=None, help="Optional device index.")
    parser.add_argument("--serial", type=str, default=None, help="Optional serial number.")
    return parser.parse_args()


def _get_cam(cal, kind: str):
    """
    Return the camera calibration object, trying multiple attribute names for compatibility.

    Older pyk4a exposes color_camera_calibration/depth_camera_calibration; newer exposes getters only.
    """
    for attr in (f"{kind}_camera_calibration", f"{kind}_camera", kind):
        if hasattr(cal, attr):
            return getattr(cal, attr)
    return None


def _to_intrinsics_from_obj(cam):
    """Parse intrinsics from the older calibration object with parameters.param."""
    p = cam.parameters.param
    return {
        "width": cam.resolution_width,
        "height": cam.resolution_height,
        "fx": p.fx,
        "fy": p.fy,
        "cx": p.cx,
        "cy": p.cy,
        "distortion": [p.k1, p.k2, p.p1, p.p2, p.k3],
    }


def _width_height_from_color(resolution_enum):
    from pyk4a import ColorResolution

    mapping = {
        ColorResolution.RES_720P: (1280, 720),
        ColorResolution.RES_1080P: (1920, 1080),
        ColorResolution.RES_1440P: (2560, 1440),
        ColorResolution.RES_1536P: (2048, 1536),
        ColorResolution.RES_2160P: (3840, 2160),
        ColorResolution.RES_3072P: (4096, 3072),
    }
    return mapping.get(resolution_enum, (1280, 720))


def _width_height_from_depth(depth_mode_enum):
    from pyk4a import DepthMode

    mapping = {
        DepthMode.NFOV_2X2BINNED: (320, 288),
        DepthMode.NFOV_UNBINNED: (640, 576),
        DepthMode.WFOV_2X2BINNED: (512, 512),
        DepthMode.WFOV_UNBINNED: (1024, 1024),
        DepthMode.PASSIVE_IR: (640, 576),
    }
    return mapping.get(depth_mode_enum, (640, 576))


def _to_intrinsics_from_methods(cal, calib_type, width, height):
    """
    Parse intrinsics using get_camera_matrix/get_distortion_coefficients API
    (pyk4a versions that do not expose *camera_calibration members).
    """
    K = np.array(cal.get_camera_matrix(calib_type), dtype=float)
    dist = np.array(cal.get_distortion_coefficients(calib_type), dtype=float).ravel().tolist()
    return {
        "width": int(width),
        "height": int(height),
        "fx": float(K[0, 0]),
        "fy": float(K[1, 1]),
        "cx": float(K[0, 2]),
        "cy": float(K[1, 2]),
        "distortion": dist,
    }


def _extrinsics_depth_to_color(cal):
    from pyk4a import CalibrationType

    # Newer API: explicit getter
    if hasattr(cal, "get_extrinsic_parameters"):
        extr = cal.get_extrinsic_parameters(CalibrationType.DEPTH, CalibrationType.COLOR)
        # Some versions return an object with .rotation/.translation; others return (R, t) tuple
        rot = getattr(extr, "rotation", None)
        trans_attr = getattr(extr, "translation", None)
        if rot is None and isinstance(extr, (list, tuple)) and len(extr) == 2:
            rot, trans_attr = extr
        R = np.array(rot, dtype=float).reshape(3, 3)
        if hasattr(trans_attr, "x"):
            trans = [trans_attr.x, trans_attr.y, trans_attr.z]
        else:
            trans = np.array(trans_attr, dtype=float).ravel().tolist()
    # Older API: calibration.extrinsics list
    elif hasattr(cal, "extrinsics"):
        extr = cal.extrinsics[1].extrinsics  # index 1 is depth, 0 is color in SDK ordering
        R = np.array(extr.rotation, dtype=float).reshape(3, 3)
        trans = [extr.translation.x, extr.translation.y, extr.translation.z]
    else:
        raise AttributeError("Cannot locate extrinsic parameters in calibration object.")

    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    # Azure Kinect reports translation in millimeters; convert to meters
    T[:3, 3] = np.array(trans, dtype=float) / 1000.0
    return T.tolist()


def main():
    args = _parse_args()
    try:
        from pyk4a import CalibrationType, ColorResolution, DepthMode, Config, PyK4A
    except ImportError as exc:  # pragma: no cover - runtime only
        print("pyk4a is not installed. Install Azure Kinect SDK and `pip install pyk4a`.", file=sys.stderr)
        raise SystemExit(1) from exc

    device_kwargs = {}
    if args.device is not None:
        device_kwargs["device_id"] = args.device
    if args.serial is not None:
        device_kwargs["serial"] = args.serial

    k4a = PyK4A(Config(), **device_kwargs)
    k4a.start()
    try:
        cal = k4a.calibration
        color_cam = _get_cam(cal, "color")
        depth_cam = _get_cam(cal, "depth")

        if color_cam is not None and depth_cam is not None:
            color = _to_intrinsics_from_obj(color_cam)
            depth = _to_intrinsics_from_obj(depth_cam)
        else:
            color_w, color_h = _width_height_from_color(cal.color_resolution)
            depth_w, depth_h = _width_height_from_depth(cal.depth_mode)
            color = _to_intrinsics_from_methods(cal, CalibrationType.COLOR, color_w, color_h)
            depth = _to_intrinsics_from_methods(cal, CalibrationType.DEPTH, depth_w, depth_h)

        # 4x4 transform from depth optical -> color optical (meters)
        T = _extrinsics_depth_to_color(cal)

        pprint(
            {
                "color_intrinsics": color,
                "depth_intrinsics": depth,
                "T_color_depth": T,
            }
        )
    finally:
        k4a.stop()


if __name__ == "__main__":
    main()
