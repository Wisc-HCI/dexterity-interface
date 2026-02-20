
from __future__ import annotations

from pathlib import Path
import logging
import os
import time

import numpy as np




_LOGGER = logging.getLogger(__name__)

_SCENE_OBJECTS = [
    {
        "name": "cup",
        "description": "Small cup",
        "pose": np.array([0.2, 0.1, 0.95, 0.0, 0.0, 0.0, 1.0]),
        "grasp_pose": np.array([0, 0.0, 0.08, 1, 0, 0, 0]),
        "dimensions": np.array([0.05, 0.05, 0.08]),
        "yolo_labels": ("cup", "mug"),
    },
    {
        "name": "bowl",
        "description": "Bowl",
        "pose": np.array([0.2, -0.2, 0.95, 0.0, 0.0, 0.0, 1.0]),
        "grasp_pose": np.array([-0.068, 0 , 0.065, 1, 0, 0, 0]), 
        "dimensions": np.array([0.136, 0.136, 0.0476]),
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
            "pose": list(obj["pose"]),
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
    return {obj["name"]: float(obj["dimensions"][2]) for obj in _SCENE_OBJECTS}


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

    if not camera or not yolo or not settings:
        _LOGGER.warning("No camera or yolo. Returning default objects")
        return _SCENE_OBJECTS
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

            print(f"Detected labels: {labels}")
            print("BEFORE FILTER y min and max", np.min(point_clouds[0][:,1]), np.max(point_clouds[1][:,1]))
            filtered_clouds = yolo.filter_point_clouds(
                point_clouds,
                z_thresh=settings["outlier_z_thresh"],
                min_points=settings["outlier_min_points"],
                min_keep_ratio=settings["outlier_min_keep_ratio"],
            )
            print("AFTEr FILTER y min and max", np.min(filtered_clouds[0][:,1]), np.max(filtered_clouds[1][:,1]))
            print("MEAN", filtered_clouds[0].mean(axis=0))
            print("MEDIAN", np.median(filtered_clouds[0], axis=0))
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
            pose = list(obj["pose"])
            grasp_pose = obj["grasp_pose"]
            dimensions = obj["dimensions"]


            obj_samples = samples.get(name, [])
            if obj_samples:
                arr = np.stack(obj_samples, axis=0)
                valid = arr[np.all(np.isfinite(arr), axis=1)]
                if valid.shape[0] >= settings["min_detections"]:
                    position = np.nanmedian(valid, axis=0)
                    # Convert z at centroid to z at bottom (isaacsim convention)
                    
                    z_pos = float(position[2]) - dimensions[2]/2 
                    # TODO: HANDLE THIS BETTER
                    # z_pos = max(z_pos, 0.95) # Ensure z is above table
                    # z_pos = 0.945 # Use fixed z for now since estimation is noisy
                    pose[:3] = [float(position[0]), float(position[1]), z_pos]
                    
                else:
                    _LOGGER.warning(
                        "Insufficient detections for %s (have %d, need %d).",
                        name,
                        valid.shape[0],
                        settings["min_detections"],
                    )
            else:
                _LOGGER.warning("No detections for %s; using default pose.", name)

            print(f"Object '{name}': pose={pose}")
            output.append({"name": name, "description": obj["description"], "pose": np.array(pose),
                           'grasp_pose': grasp_pose, 'dimensions': dimensions})

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

    strict = _bool_env("DEXTERITY_SCENE_STRICT", default=False)

    try:
        localized = _localize_scene(camera,  yolo, settings)
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
