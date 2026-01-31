from __future__ import annotations

from planning.examples.rgbd_yolo_stream import (
     parse_args, _DEFAULT_CONFIGS, _resolution_from_config, _init_camera, YoloPerception, _label_colors, 
       _update_pointcloud_plot
    )
import time
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import open3d as o3d


# TODO: MAKE THIS BETTER
POINTCLOUD_PATH = Path("pointcloud_output")
POINTCLOUD_PATH.mkdir(parents=True, exist_ok=True)

def main():
    """
    Run the RGB-D streaming loop with YOLO segmentation and visualization.

    Returns:
        None
    """
    args = parse_args()

    camera_cfg = args.camera_config
    if camera_cfg is None:
        camera_cfg = _DEFAULT_CONFIGS.get(args.camera)
    if camera_cfg is None:
        raise FileNotFoundError(f"No default config found for camera {args.camera}")
    camera_cfg = camera_cfg.expanduser().resolve()
    if not camera_cfg.exists():
        raise FileNotFoundError(f"Camera config not found: {camera_cfg}")

    resolution = _resolution_from_config(camera_cfg)
    print(f"Starting {args.camera} with resolution {resolution}, fps={args.fps}, align={args.align}")
    camera = _init_camera(
        args.camera,
        camera_cfg,
        resolution=resolution,
        fps=args.fps,
        align=args.align,
        serial=args.serial,
        device_index=args.device_index,
    )

    transform_kwargs = YoloPerception.load_constructor_kwargs(args.transform_config)
    yolo = YoloPerception(
        camera,
        model_path=args.model,
        conf=args.confidence,
        iou=args.iou,
        classes=args.classes,
        device=args.device,
        **transform_kwargs,
    )

    plt.ion()
    fig = plt.figure(figsize=(7, 5))
    ax = fig.add_subplot(111, projection="3d")

    try:
        while True:
            try:
                frame = camera.latest()
            except RuntimeError:
                time.sleep(0.01)
                continue

            if frame.color is None or frame.depth is None:
                time.sleep(0.01)
                continue


            rgb = frame.color
            depth_m = frame.depth

            semantic_mask, labels = yolo.detect_rgb(rgb)
            point_clouds, labels, depth_mask = yolo.get_object_point_clouds(
                depth_m, semantic_mask, labels, return_depth_mask=True
            )

            for i, pc_np in enumerate(point_clouds):
                # Save each object pointcloud
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(pc_np)
                object_name = labels[i]
                f_path = Path(POINTCLOUD_PATH) / f"{object_name}.xyz"

                o3d.io.write_point_cloud(f_path, pcd, write_ascii= True) 

            POINTCLOUD_PATH

            centroids = yolo.get_centroid(point_clouds)
            colors = _label_colors(len(labels))

            if len(labels) > 0:
                _update_pointcloud_plot(ax, point_clouds, centroids, labels, colors, max_points=args.max_points)
                plt.pause(0.001)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        plt.close("all")


if __name__ == "__main__":
    main()
