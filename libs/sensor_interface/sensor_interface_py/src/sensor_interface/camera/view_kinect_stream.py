from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

from sensor_interface.camera.rgbd_camera import CameraIntrinsics
from sensor_interface.camera.kinect_interface import KinectInterface


def _make_placeholder_intrinsics(width: int, height: int) -> CameraIntrinsics:
    """
    Create placeholder camera intrinsics with approximate focal lengths.

    Args:
        width (int): Image width in pixels.
        height (int): Image height in pixels.

    Returns:
        CameraIntrinsics: Intrinsics object with
            - fx, fy ≈ 600.0 (placeholder focal lengths),
            - cx = width / 2.0 (principal point x),
            - cy = height / 2.0 (principal point y).
    """
    fx = fy = 600.0
    cx = width / 2.0
    cy = height / 2.0
    return CameraIntrinsics(width=width, height=height, fx=fx, fy=fy, cx=cx, cy=cy)


def main() -> None:
    """
    Entry point for Azure Kinect demo.

    Parses command-line arguments, configures intrinsics, starts the Kinect device,
    and displays synchronized color and depth streams in OpenCV windows.

    Command-line Args:
        --width (int): Desired stream width (default 1280).
        --height (int): Desired stream height (default 720).
        --fps (int): Target frames per second {5, 15, 30} (default 30).
        --align (str): Alignment mode {"color", "depth"} (default "color").
        --serial (str): Optional Azure Kinect serial number to select device.

    Behavior:
        - Displays color frames in a window as BGR images.
        - Displays depth frames as a colormap (0.2–4.0 m clipped).
        - Press "q" or Ctrl+C to exit.

    Returns:
        None
    """
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--align", type=str, default="color", choices=["color", "depth"])
    ap.add_argument("--serial", type=str, default=None, help="Optional Azure Kinect serial")
    args = ap.parse_args()

    W, H = args.width, args.height
    color_K = _make_placeholder_intrinsics(W, H)
    depth_K = _make_placeholder_intrinsics(W, H)
    T_color_depth = np.eye(4, dtype=np.float32)  # replace with calibrated extrinsics if available

    cam = KinectInterface(color_intrinsics=color_K, depth_intrinsics=depth_K, T_color_depth=T_color_depth)

    print("Starting Azure Kinect...")
    cam.start(resolution=(W, H), fps=args.fps, align=args.align, serial=args.serial)

    # Warm-up a short period for auto-exposure/white-balance
    time.sleep(0.2)

    try:
        while True:
            # Latest non-blocking frame (raises RuntimeError until first frame is available)
            frame = cam.latest()
            color_rgb = frame.color  # (H, W, 3) uint8 RGB image
            depth_m = frame.depth    # (H, W) float32 depth in meters

            # Convert RGB -> BGR for OpenCV display
            bgr = cv2.cvtColor(color_rgb, cv2.COLOR_RGB2BGR) if color_rgb is not None else None

            # Depth visualization: meters -> [0.2, 4.0] m -> colormap
            if depth_m is not None:
                depth_vis = np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)
                depth_vis = np.clip(depth_vis, 0.2, 4.0)
                depth_norm = ((depth_vis - 0.2) / (4.0 - 0.2) * 255.0).astype(np.uint8)
                depth_cmap = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
                cv2.imshow("Kinect Depth (m, colormap)", depth_cmap)

            if bgr is not None:
                cv2.imshow("Kinect Color (BGR)", bgr)

            # Exit on 'q'
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f"[Error] {e}")
    finally:
        print("Stopping Azure Kinect...")
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
