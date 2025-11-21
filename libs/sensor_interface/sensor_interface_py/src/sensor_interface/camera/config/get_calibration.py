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
    """Return the camera calibration object, trying multiple attribute names for compatibility."""
    for attr in (f"{kind}_camera_calibration", f"{kind}_camera", kind):
        if hasattr(cal, attr):
            return getattr(cal, attr)
    raise AttributeError(
        f"Calibration object missing expected {kind} camera calibration. "
        f"Available attributes: {dir(cal)}"
    )


def _to_intrinsics(cam):
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


def main():
    args = _parse_args()
    try:
        from pyk4a import Config, PyK4A
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

        color = _to_intrinsics(color_cam)
        depth = _to_intrinsics(depth_cam)

        # 4x4 transform from depth optical -> color optical (meters)
        extr = cal.extrinsics[1].extrinsics  # index 1 is depth, 0 is color in SDK ordering
        T = np.eye(4, dtype=float)
        R = extr.rotation
        T[:3, :3] = [
            [R[0], R[1], R[2]],
            [R[3], R[4], R[5]],
            [R[6], R[7], R[8]],
        ]
        T[:3, 3] = [
            extr.translation.x / 1000.0,
            extr.translation.y / 1000.0,
            extr.translation.z / 1000.0,
        ]

        pprint(
            {
                "color_intrinsics": color,
                "depth_intrinsics": depth,
                "T_color_depth": T.tolist(),
            }
        )
    finally:
        k4a.stop()


if __name__ == "__main__":
    main()
