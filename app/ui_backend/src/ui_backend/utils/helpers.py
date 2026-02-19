
from __future__ import annotations

from pathlib import Path
import logging
import os
import time

import numpy as np
import math
import random




# def _sample_within_cylinder(position:list, radius:float, height: float):
#     """
#     Generate a random location within a cylinder at positioned at the
#     given position with the given radius and height.
#     Args:
#         position(list): (3,) position of cylinder [x,y,z] in m where x,y are at center
#         of cylinder and z is at its base.
#         radius (float): Radius of the cylinder in m
#         height (float): Height of cylinder in m
#     """
#     r = radius * math.sqrt(random.random())
#     theta = random.uniform(0, 2 * math.pi)

#     x = position[0] + r * math.cos(theta)
#     y = position[1] + r * math.sin(theta)
#     z = position[2] + random.uniform(0.0, height)

#     return [x, y, z, 0.0, 0.0, 0.0, 1.0]


def get_current_scene(all_objects:bool=False):
    """
    Returns the current scene description for planning and execution.
    Args:
        all_objects (bool): True if return all objects in the scene. False if only 
        return the major objects (used for LLM), not the minor/filling ones.
    Returns:
        (list[dict]): List of object dictionaries with the form:  {'name': ..., 'description': ..., 'pose': ..., grasp_pose: ..., dimensions: ....}
            - pose (np.ndarray): (7,)  Centroid of object (with z at the bottom of object) in [x,y,z, qx, qy, qz, qw] in m
            - grasp_pose (np.ndarray): (7,) Pose to grasp relative to centroid [x,y,z, qx, qy, qz, qw] in m.
            - dimensions (np.ndarray): (3,) [x (width), y (length), z (height)] in m
    """

    # TODO: Instead treat grasp_pose as a couple points
    return [

        # TOP grasp
        {"name": "cup", "description": "Small cup", "pose": np.array([0.3, 0.25, 0.94, 0.0, 0.0, 0.0, 1.0]),
         "grasp_pose": np.array([0, 0.0, 0.08, 0.707, 0.707, 0, 0]), 
         "dimensions": np.array([0.05, 0.05, 0.08])},
        {"name": "cup_1", "description": "Small cup", "pose": np.array([0.2, 0.25, 0.94, 0.0, 0.0, 0.0, 1.0]),
         "grasp_pose": np.array([0, 0.0, 0.08, 0.707, 0.707, 0, 0]), 
         "dimensions": np.array([0.05, 0.05, 0.08])},
        {"name": "cup_2", "description": "Small cup", "pose": np.array([0.1, 0.25, 0.94, 0.0, 0.0, 0.0, 1.0]),
         "grasp_pose": np.array([0, 0.0, 0.08, 0.707, 0.707, 0, 0]), 
         "dimensions": np.array([0.05, 0.05, 0.08])},


        # Pincer grasp
        {"name": "bowl", "description": "Bowl.", "pose": np.array([ -0.1, 0.25, 0.9369, 0.0, 0.0, 0.0, 1.0]),
         "grasp_pose": np.array([0, 0.068, 0.08, 0.707, 0.707, 0, 0]), 
         "dimensions": np.array([0.136, 0.136, 0.0476])},
        {"name": "bowl_1", "description": "Bowl.", "pose": np.array([ -0.3, 0.25, 0.9369, 0.0, 0.0, 0.0, 1.0]),
         "grasp_pose": np.array([0, 0.068, 0.08, 0.707, 0.707, 0, 0]), 
         "dimensions": np.array([0.136, 0.136, 0.0476])}
    ]


    # CUP_POS = [0.2, 0.1, 0.95]
    # CUP_RADIUS = 0.02
    # CUP_HEIGHT = 0.05
    # primary = [
    #     {"name": "cup", "description": "Small cup. Height: 0.08 (m). Use this grasp pose: [0.2, 0.11, 1, 0, -0.818, 0.574, 0]", 
    #         "pose": CUP_POS + [0.0, 0.0, 0.0, 1.0]},
    #     {"name": "bowl", "description": "Bowl. Height: 0.0476, Width:0.18 (m). Pour location: [0.2, -0.1, 1.1, 0, -0.788, -0.614, 0]", 
    #         "pose": [ 0.2, -0.2, 0.95, 0.0, 0.0, 0.0, 1.0]},
    # ]

    # filler  = []

    # # for i in range(30):
    # #     filler.append( 
    # #         {"name": f"cube_{i}", 
    # #          "description": "Sugar cube", 
    # #          "pose": _sample_within_cylinder(CUP_POS,CUP_RADIUS, CUP_HEIGHT)}
    # #     )

    # if all_objects:
    #     return primary + filler
    # else:
    #     return primary

_LOGGER = logging.getLogger(__name__)

_SCENE_OBJECTS = [
    {
        "name": "cup",
        "description": (
            "Small cup. Height: 0.08 (m). Use this grasp pose: "
            "[0.2, 0.11, 1, 0, -0.818, 0.574, 0]"
        ),
        "default_pose": [0.2, 0.1, 0.95, 0.0, 0.0, 0.0, 1.0],
        "height": 0.08,
        "yolo_labels": ("cup", "mug"),
    },
    {
        "name": "bowl",
        "description": (
            "Bowl. Height: 0.0476, Width:0.18 (m). Pour location: "
            "[0.2, -0.2, 1.15, 0, -0.788, -0.614, 0]"
        ),
        "default_pose": [0.2, -0.2, 0.95, 0.0, 0.0, 0.0, 1.0],
        "height": 0.0476,
        "yolo_labels": ("bowl",),
    },
]


def _repo_root() -> Path:
    here = Path(__file__).resolve()
    for parent in [here] + list(here.parents):
        if (parent / "libs").is_dir() and (parent / "app").is_dir():
            return parent
    return Path.cwd()


def _bool_env(name: str, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _resolve_path(value: str | None, default: Path, root: Path) -> Path:
    if value:
        path = Path(value).expanduser()
        if not path.is_absolute():
            path = (root / path).resolve()
        return path
    return default


def _default_scene() -> list[dict]:
    return [
        {
            "name": obj["name"],
            "description": obj["description"],
            "pose": list(obj["default_pose"]),
        }
        for obj in _SCENE_OBJECTS
    ]


def _localization_settings() -> dict:
    root = _repo_root()

    camera_name = os.getenv("DEXTERITY_CAMERA", "realsense").strip().lower()
    if camera_name not in {"realsense", "kinect"}:
        raise ValueError("DEXTERITY_CAMERA must be 'realsense' or 'kinect'")

    camera_default = root / "libs" / "sensor_interface" / "sensor_interface_py" / "src" / "sensor_interface" / "camera" / "config"
    if camera_name == "realsense":
        default_camera_cfg = camera_default / "realsense_config.yaml"
    else:
        default_camera_cfg = camera_default / "kinect_config.yaml"

    default_transform_cfg = (
        camera_default / "table_world_transform.yaml"
        if (camera_default / "table_world_transform.yaml").exists()
        else root / "libs" / "planning" / "planning_py" / "src" / "planning" / "config" / "camera_transforms.yaml"
    )

    default_model = root / "libs" / "planning" / "planning_py" / "src" / "planning" / "yolo11n-seg.pt"

    camera_config = _resolve_path(os.getenv("DEXTERITY_CAMERA_CONFIG"), default_camera_cfg, root)
    transform_config = _resolve_path(os.getenv("DEXTERITY_CAMERA_TRANSFORM"), default_transform_cfg, root)
    yolo_model = _resolve_path(os.getenv("DEXTERITY_YOLO_MODEL"), default_model, root)

    classes_raw = os.getenv("DEXTERITY_YOLO_CLASSES")
    classes = None
    if classes_raw:
        classes = [int(v) for v in classes_raw.replace(",", " ").split() if v.strip()]

    return {
        "camera_name": camera_name,
        "camera_config": camera_config,
        "camera_align": os.getenv("DEXTERITY_CAMERA_ALIGN", "color").strip().lower(),
        "camera_fps": int(os.getenv("DEXTERITY_CAMERA_FPS", "30")),
        "camera_serial": os.getenv("DEXTERITY_CAMERA_SERIAL") or None,
        "kinect_device": os.getenv("DEXTERITY_KINECT_DEVICE") or None,
        "yolo_model": yolo_model,
        "yolo_conf": float(os.getenv("DEXTERITY_YOLO_CONF", "0.3")),
        "yolo_iou": float(os.getenv("DEXTERITY_YOLO_IOU", "0.45")),
        "yolo_device": os.getenv("DEXTERITY_YOLO_DEVICE") or None,
        "yolo_classes": classes,
        "transform_config": transform_config,
        "frames": int(os.getenv("DEXTERITY_LOCALIZATION_FRAMES", "5")),
        "warmup": int(os.getenv("DEXTERITY_LOCALIZATION_WARMUP", "5")),
        "timeout_s": float(os.getenv("DEXTERITY_LOCALIZATION_TIMEOUT", "5.0")),
        "min_detections": int(os.getenv("DEXTERITY_LOCALIZATION_MIN_DETECTIONS", "1")),
        "min_object_points": int(os.getenv("DEXTERITY_MIN_OBJECT_POINTS", "30")),
        "outlier_z_thresh": float(os.getenv("DEXTERITY_OUTLIER_Z", "3.5")),
        "outlier_min_points": int(os.getenv("DEXTERITY_OUTLIER_MIN_POINTS", "30")),
        "outlier_min_keep_ratio": float(os.getenv("DEXTERITY_OUTLIER_MIN_KEEP_RATIO", "0.3")),
        "z_mode": os.getenv("DEXTERITY_Z_MODE", "bottom").strip().lower(),
        "z_percentile": float(os.getenv("DEXTERITY_Z_PERCENTILE", "5.0")),
    }


def _init_camera(settings: dict):
    camera_config = settings["camera_config"]
    if not camera_config.exists():
        raise FileNotFoundError(f"Camera config not found: {camera_config}")

    camera_name = settings["camera_name"]
    align = settings["camera_align"]
    fps = settings["camera_fps"]
    serial = settings["camera_serial"] or None

    if camera_name == "realsense":
        from sensor_interface.camera.realsense_interface import RealsenseInterface

        camera = RealsenseInterface.from_yaml(str(camera_config))
        resolution = (camera.color_intrinsics.width, camera.color_intrinsics.height)
        camera.start(resolution=resolution, fps=fps, align=align, serial=serial)
        return camera

    from sensor_interface.camera.kinect_interface import KinectInterface

    camera = KinectInterface.from_yaml(str(camera_config))
    resolution = (camera.color_intrinsics.width, camera.color_intrinsics.height)
    device = settings["kinect_device"]
    device = int(device) if device is not None else None
    camera.start(resolution=resolution, fps=fps, align=align, device=device, serial=serial)
    return camera


def _init_yolo(camera, settings: dict):
    from planning.perception.yolo_perception import YoloPerception

    transform_config = settings["transform_config"]
    if not transform_config.exists():
        _LOGGER.warning("Transform config not found (%s); using identity transform.", transform_config)

    transform_kwargs = YoloPerception.load_constructor_kwargs(transform_config)
    return YoloPerception(
        camera,
        model_path=settings["yolo_model"],
        conf=settings["yolo_conf"],
        iou=settings["yolo_iou"],
        classes=settings["yolo_classes"],
        device=settings["yolo_device"],
        **transform_kwargs,
    )


def _collect_frames(camera, *, frames: int, warmup: int, timeout_s: float) -> list:
    collected = []
    start = time.time()

    while len(collected) < frames and (time.time() - start) < timeout_s:
        try:
            frame = camera.latest()
        except RuntimeError:
            time.sleep(0.01)
            continue

        if frame.color is None or frame.depth is None:
            time.sleep(0.01)
            continue

        if warmup > 0:
            warmup -= 1
            time.sleep(0.01)
            continue

        collected.append(frame)
        time.sleep(0.01)
        

    return collected


def _label_map() -> dict[str, str]:
    mapping: dict[str, str] = {}
    for obj in _SCENE_OBJECTS:
        for label in obj["yolo_labels"]:
            mapping[label.lower()] = obj["name"]
    return mapping


def _object_heights() -> dict[str, float]:
    return {obj["name"]: float(obj["height"]) for obj in _SCENE_OBJECTS}


def _estimate_object_position(
    centroid: np.ndarray,
    points: np.ndarray,
    height: float | None,
    z_mode: str,
    z_percentile: float,
) -> np.ndarray:
    if centroid is None or not np.all(np.isfinite(centroid)):
        return centroid

    position = np.array(centroid, dtype=np.float32, copy=True)
    if z_mode != "bottom" or height is None or points is None or points.size == 0:
        return position

    z_vals = points[:, 2]
    if z_vals.size == 0:
        return position

    z_bottom = float(np.nanpercentile(z_vals, z_percentile))
    if not np.isfinite(z_bottom):
        return position

    position[2] = z_bottom + 0.5 * float(height)
    return position


def _localize_scene(camera,  yolo, settings) -> list[dict] | None:

    try:


        frames = _collect_frames(
            camera,
            frames=settings["frames"],
            warmup=settings["warmup"],
            timeout_s=settings["timeout_s"],
        )
        if not frames:
            _LOGGER.warning("No frames collected for localization.")
            return None

        label_map = _label_map()
        heights = _object_heights()
        samples: dict[str, list[np.ndarray]] = {obj["name"]: [] for obj in _SCENE_OBJECTS}

        for frame in frames:
            semantic_mask, labels = yolo.detect_rgb(frame.color)
            if not labels:
                continue

            point_clouds, labels = yolo.get_object_point_clouds(frame.depth, semantic_mask, labels)
            filtered_clouds = yolo.filter_point_clouds(
                point_clouds,
                z_thresh=settings["outlier_z_thresh"],
                min_points=settings["outlier_min_points"],
                min_keep_ratio=settings["outlier_min_keep_ratio"],
            )
            centroids = yolo.get_centroid(filtered_clouds, filter_outliers=False)

            frame_best: dict[str, dict] = {}
            for idx, label in enumerate(labels):
                if label is None:
                    continue

                obj_name = label_map.get(str(label).strip().lower())
                if obj_name is None:
                    continue

                pc = filtered_clouds[idx]
                if pc is None or pc.size == 0:
                    continue
                if pc.shape[0] < settings["min_object_points"]:
                    continue

                centroid = centroids[idx]
                if centroid is None or not np.all(np.isfinite(centroid)):
                    continue

                size = int(pc.shape[0])
                prev = frame_best.get(obj_name)
                if prev is None or size > prev["size"]:
                    position = _estimate_object_position(
                        centroid,
                        pc,
                        heights.get(obj_name),
                        settings["z_mode"],
                        settings["z_percentile"],
                    )
                    frame_best[obj_name] = {"position": position, "size": size}

            for obj_name, data in frame_best.items():
                samples[obj_name].append(data["position"])

        output: list[dict] = []
        for obj in _SCENE_OBJECTS:
            name = obj["name"]
            pose = list(obj["default_pose"])

            obj_samples = samples.get(name, [])
            if obj_samples:
                arr = np.stack(obj_samples, axis=0)
                valid = arr[np.all(np.isfinite(arr), axis=1)]
                if valid.shape[0] >= settings["min_detections"]:
                    position = np.nanmedian(valid, axis=0)
                    pose[:3] = [float(position[0]), float(position[1]), float(position[2])]
                else:
                    _LOGGER.warning(
                        "Insufficient detections for %s (have %d, need %d).",
                        name,
                        valid.shape[0],
                        settings["min_detections"],
                    )
            else:
                _LOGGER.warning("No detections for %s; using default pose.", name)

            output.append({"name": name, "description": obj["description"], "pose": pose})

        return output
    finally:

        if camera is not None:
            try:
                # camera.stop()
                pass
            except Exception:
                pass


def get_current_scene(camera,  yolo, settings) -> list[dict]:
    """
    Returns the current scene description for planning and execution.

    Returns:
        list[dict]: List of object dictionaries with the form:
            {'name': ..., 'description': ..., 'pose': ...}
    """
    if os.getenv("DEXTERITY_SCENE_SOURCE", "yolo").strip().lower() in {"static", "hardcoded"}:
        return _default_scene()

    strict = _bool_env("DEXTERITY_SCENE_STRICT", default=False)

    try:
        localized = _localize_scene(camera,  yolo, settings)
        print("LOCALIZED SCENE:", localized)
    except Exception as exc:
        if strict:
            raise
        _LOGGER.warning("Scene localization failed; using default poses. Error: %s", exc, exc_info=True)
        return _default_scene()

    if localized is None:
        if strict:
            raise RuntimeError("Scene localization returned no results.")
        _LOGGER.warning("Scene localization returned no results; using default poses.")
        return _default_scene()

    return localized
