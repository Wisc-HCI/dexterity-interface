"""
Example: Live RealSense stream using RealsenseInterface + OpenCV.

Run:
    python libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/examples/realsense_stream_example.py
"""

import os
import time
import cv2
import numpy as np

from sensor_interface.camera.realsense_interface import RealsenseInterface


def main():
    cur_dir = os.path.dirname(__file__)
    config_path = os.path.join(
        cur_dir, "..", "config", "realsense_config.yaml"
    )
    config_path = os.path.abspath(config_path)

    camera = RealsenseInterface.from_yaml(config_path)
    camera.start(resolution=(640, 480), fps=30, align="color")

    print("Streaming... press 'q' to quit.")

    try:
        while True:
            frame = camera.latest()

            color = frame.color
            depth = frame.depth

            if color is None or depth is None:
                continue

            # OpenCV uses BGR; convert from RGB for correct colors
            color_bgr = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)

            # Depth visualization (normalize to 0-255)
            depth_vis = depth.copy()
            valid_mask = np.isfinite(depth_vis) & (depth_vis > 0)
            if np.any(valid_mask):
                dmin = float(np.min(depth_vis[valid_mask]))
                dmax = float(np.max(depth_vis[valid_mask]))
                if dmax > dmin:
                    depth_vis = (depth_vis - dmin) / (dmax - dmin)
            depth_vis = (depth_vis * 255.0).clip(0, 255).astype(np.uint8)
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

            cv2.imshow("RealSense Color", color_bgr)
            cv2.imshow("RealSense Depth (vis)", depth_vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            time.sleep(0.0)

    finally:
        camera.stop()
        cv2.destroyAllWindows()
        print("Stopped.")


if __name__ == "__main__":
    main()
