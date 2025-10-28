from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

from sensor_interface.camera.rgbd_camera import CameraIntrinsics
from sensor_interface.camera.realsense_interface import RealsenseInterface


def _make_intrinsics(width: int, height: int) -> CameraIntrinsics:
    """
    Create placeholder pinhole intrinsics for a given image size.

    Args:
        width (int): Image width in pixels.
        height (int): Image height in pixels.

    Returns:
        (CameraIntrinsics): Intrinsics with fx=fy=600 and cx,cy at image center.
    """
    fx = fy = 600.0
    cx = width / 2.0
    cy = height / 2.0
    return CameraIntrinsics(width=width, height=height, fx=fx, fy=fy, cx=cx, cy=cy)


def _depth_to_colormap(depth_m: np.ndarray, min_m: float = 0.2, max_m: float = 3.0) -> np.ndarray:
    """
    Convert a depth map in meters to an 8-bit BGR colormap for visualization.

    Args:
        depth_m (np.ndarray): (H, W) float32 depth in meters.
        min_m (float): Minimum clipping distance in meters.
        max_m (float): Maximum clipping distance in meters.

    Returns:
        (np.ndarray): (H, W, 3) uint8 BGR image suitable for cv2.imshow.
    """
    depth = np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)
    depth = np.clip(depth, min_m, max_m)
    norm = ((depth - min_m) / (max_m - min_m) * 255.0).astype(np.uint8)
    return cv2.applyColorMap(norm, cv2.COLORMAP_JET)


def main() -> None:
    """
    Live RealSense preview (RGB + depth) using OpenCV. Press 'q' to quit.

    Args:
        (no args; CLI flags parsed internally)

    Returns:
        (None)
    """
    ap = argparse.ArgumentParser(description="RealSense live viewer (OpenCV). Press 'q' to quit.")
    ap.add_argument("--width", type=int, default=640, help="Stream width in pixels.")
    ap.add_argument("--height", type=int, default=480, help="Stream height in pixels.")
    ap.add_argument("--fps", type=int, default=30, help="Target frames per second.")
    ap.add_argument("--align", choices=["color", "depth"], default="color",
                    help='Align depth->color ("color") or color->depth ("depth").')
    ap.add_argument("--serial", type=str, default=None, help="Camera serial if multiple devices are attached.")
    args = ap.parse_args()

    W, H = args.width, args.height
    Kc = _make_intrinsics(W, H)
    Kd = _make_intrinsics(W, H)
    T_color_depth = np.eye(4, dtype=np.float32)  # replace with calibrated extrinsics when available

    cam = RealsenseInterface(color_intrinsics=Kc, depth_intrinsics=Kd, T_color_depth=T_color_depth)
    print("Starting RealSense...")
    cam.start(resolution=(W, H), fps=args.fps, align=args.align, serial=args.serial)

    # small warmup in case frames aren't ready immediately
    t0 = time.time()
    try:
        while True:
            try:
                frame = cam.latest()
            except RuntimeError:
                if time.time() - t0 < 2.0:
                    time.sleep(0.01)
                    continue
                raise  # if it's been a while, surface the error

            if frame.color is not None:
                bgr = cv2.cvtColor(frame.color, cv2.COLOR_RGB2BGR)
                cv2.imshow("RealSense Color (BGR)", bgr)

            if frame.depth is not None:
                depth_vis = _depth_to_colormap(frame.depth, 0.2, 3.0)
                cv2.imshow("RealSense Depth (m, colormap)", depth_vis)

            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
    finally:
        print("Stopping RealSense...")
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
