"""
Example: Live RealSense RGB-D stream using RealsenseInterface + OpenCV.

This script shows how to:
1. Load a RealSense configuration from YAML,
2. Start the camera and wait until the first frame is available,
3. Display aligned RGB + depth frames,
4. Exit cleanly using 'q' or Ctrl-C.

Run:
    python3 -m sensor_interface.camera.examples.realsense_stream_example
"""

import os
import time
import cv2
import numpy as np

from sensor_interface.camera.realsense_interface import RealsenseInterface


def main():
    """
    Run a live RealSense RGB-D visualization loop.

    This function:
    - Initializes the camera from realsense_config.yaml,
    - Starts the RealSense pipeline + alignment,
    - Waits for the first valid frame (instead of raising RuntimeError),
    - Visualizes RGB and depth using OpenCV,
    - Gracefully exits on 'q' or Ctrl-C.
    """
    
    cur_dir = os.path.dirname(__file__)
    config_path = os.path.join(
        cur_dir, "..", "config", "realsense_config.yaml"
    )
    config_path = os.path.abspath(config_path)

    camera = RealsenseInterface.from_yaml(config_path)
    camera.start(resolution=(640, 480), fps=30, align="color")

    print("Starting stream...")

    frame = None
    start_time = time.time()
    while frame is None:
        try:
            frame = camera.latest()
        except RuntimeError:
            frame = None

        if time.time() - start_time > 3.0:
            camera.stop()
            raise RuntimeError("Camera failed to provide frames after 3 seconds.")

        time.sleep(0.02)

    print("Streaming... Press 'q' or Ctrl-C to quit.")

    # Visualization loop
    try:
        while True:
            try:
                frame = camera.latest()
            except RuntimeError:
                continue

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

            key = cv2.waitKey(1)
            if key & 0xFF == ord("q"):
                print("Exiting (q pressed).")
                break

    except KeyboardInterrupt:
        print("Exiting (Ctrl-C).")

    finally:
        camera.stop()
        cv2.destroyAllWindows()
        print("Stopped RealSense stream.")


if __name__ == "__main__":
    main()
