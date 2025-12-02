"""
Live streaming example for the KinectInterface.

Usage (from repo root):
    python3 -m sensor_interface.camera.examples.kinect_stream \
        --config libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/kinect_config.yaml

Requires:
    - Azure Kinect SDK installed on the system
    - `pip install pyk4a opencv-python`
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import cv2
import numpy as np


_THIS_FILE = Path(__file__).resolve()
_SENSOR_SRC = _THIS_FILE.parents[3]
if str(_SENSOR_SRC) not in sys.path:
    sys.path.append(str(_SENSOR_SRC))


from sensor_interface.camera.kinect_interface import KinectInterface


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="View Kinect RGB-D stream.")
    default_cfg = (
        _SENSOR_SRC
        / "sensor_interface"
        / "camera"
        / "config"
        / "kinect_config.yaml"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=default_cfg,
        help="Path to YAML with intrinsics/extrinsics.",
    )
    parser.add_argument(
        "--fps", type=int, default=30, help="Requested camera FPS (5, 15, or 30)."
    )
    parser.add_argument(
        "--align",
        choices=["color", "depth"],
        default="color",
        help="Align depth into color frame (default) or color into depth frame.",
    )
    parser.add_argument(
        "--device",
        type=int,
        default=None,
        help="Optional device index if multiple Kinects are present.",
    )
    parser.add_argument(
        "--max-depth",
        type=float,
        default=4.0,
        help="Clamp depth visualization to this range in meters.",
    )
    return parser.parse_args()


def _colorize_depth(depth_m: np.ndarray, max_depth: float) -> np.ndarray:
    """Create a false-color depth visualization in BGR for OpenCV display."""
    depth = np.nan_to_num(depth_m, nan=0.0, posinf=max_depth, neginf=0.0)
    depth = np.clip(depth, 0.0, max_depth)
    normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
    normalized = normalized.astype(np.uint8)
    return cv2.applyColorMap(normalized, cv2.COLORMAP_PLASMA)


def main():
    """Stream and visualize Kinect color and depth until Esc/q or window close."""
    args = _parse_args()
    camera = KinectInterface.from_yaml(str(args.config))

    # Use the calibrated color resolution from the config unless overridden.
    resolution = (camera.color_intrinsics.width, camera.color_intrinsics.height)

    camera.start(
        resolution=resolution,
        fps=args.fps,
        align=args.align,
        device=args.device,
    )

    try:
        while True:
            try:
                frame = camera.latest()
            except RuntimeError:
                # No frame yet; small sleep to avoid a tight spin.
                time.sleep(0.01)
                continue

            if frame.color is not None:
                bgr = cv2.cvtColor(frame.color, cv2.COLOR_RGB2BGR)
                cv2.imshow("Kinect Color", bgr)

            if frame.depth is not None:
                depth_vis = _colorize_depth(frame.depth, max_depth=args.max_depth)
                cv2.imshow("Kinect Depth (m)", depth_vis)

            key = cv2.waitKey(1)
            if key != -1 and (key in (27, ord("q"), ord("Q")) or (key & 0xFF) in (27, ord("q"), ord("Q"))):
                break

            # Allow closing the window via the titlebar close button
            if frame.color is not None and cv2.getWindowProperty("Kinect Color", cv2.WND_PROP_VISIBLE) < 1:
                break
            if frame.depth is not None and cv2.getWindowProperty("Kinect Depth (m)", cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
