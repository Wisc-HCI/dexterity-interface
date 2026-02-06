# Task README: Camera-to-World Localization for UI Backend

This README documents the task to replace hard-coded object positions with YOLO-based localization,
set up the camera-to-world transform, and validate object placement in IsaacSim.

It is scoped to the perception pipeline used by `app/ui_backend/src/ui_backend/utils/helpers.py`.

## Goal
- Measure and apply the camera-to-world transform (world frame = floor at center of table).
- Replace hard-coded positions in `get_current_scene()` with YOLO localization (done in helpers).
- Filter out outliers to improve centroid accuracy.
- Validate that bowl/cup appear in IsaacSim with ~2 cm accuracy at least 70% of the time.

## Key Files
- Camera world transform (update this):
  `libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/table_world_transform.yaml`
- Camera intrinsics/extrinsics:
  `libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/realsense_config.yaml`
  `libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/kinect_config.yaml`
- Localization entry point:
  `app/ui_backend/src/ui_backend/utils/helpers.py`
- Perception utilities:
  `libs/planning/planning_py/src/planning/perception/perception.py`
  `libs/planning/planning_py/src/planning/perception/yolo_perception.py`

## 1) Measure and Set the Camera-to-World Transform
Update `table_world_transform.yaml` with `T_world_color` (4x4) that maps points from the
color optical frame to the world frame (floor at center of table). Units are meters.

File:
```
libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/table_world_transform.yaml
```

Notes:
- If you already have a measured transform, paste it directly.
- If you are recalibrating: mark the table center on the floor, capture a frame,
  and estimate the camera pose relative to that origin. Then invert if needed.

## 2) Quick Visual Sanity: YOLO RGB-D Stream
This confirms segmentation, point clouds, and centroids before touching the UI backend.

RealSense:
```bash
python3 -m libs.planning.planning_py.src.planning.examples.rgbd_yolo_stream \
  --camera realsense \
  --align color \
  --camera-config libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/realsense_config.yaml \
  --transform-config libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/table_world_transform.yaml \
  --model libs/planning/planning_py/src/planning/yolo11n-seg.pt
```

Kinect:
```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export QT_QPA_PLATFORM=xcb
python3 -m libs.planning.planning_py.src.planning.examples.rgbd_yolo_stream \
  --camera kinect \
  --align color \
  --camera-config libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/kinect_config.yaml \
  --transform-config libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/table_world_transform.yaml \
  --model libs/planning/planning_py/src/planning/yolo11n-seg.pt
```

## 3) Smoke Test `get_current_scene()`
This validates that the UI backend returns YOLO-based object poses.

```bash
# If you're inside the IsaacSim container, append to PYTHONPATH (don't overwrite it),
# so prebundled deps (like pyyaml) remain visible.
export PYTHONPATH="${PYTHONPATH}:app/ui_backend/src:libs/planning/planning_py/src:libs/sensor_interface/sensor_interface_py/src"

DEXTERITY_SCENE_SOURCE=yolo \
DEXTERITY_SCENE_STRICT=1 \
DEXTERITY_CAMERA=realsense \
DEXTERITY_CAMERA_ALIGN=none \
DEXTERITY_CAMERA_CONFIG=libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/realsense_config.yaml \
DEXTERITY_CAMERA_TRANSFORM=libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/table_world_transform.yaml \
python - <<'PY'
from ui_backend.utils.helpers import get_current_scene
import json
print(json.dumps(get_current_scene(), indent=2))
PY
```

Swap `realsense` for `kinect` as needed.

## 4) End-to-End Test in IsaacSim
Make sure IsaacSim is running (e.g., `ros2 launch primitives_ros sim.launch.py`).

1) Start the IsaacSim object interface ROS node **inside the `isaac-base` container** (If you have followed the root README to setup the container and launch simulation on computer with GPU, skip this step):
```bash
sudo docker compose -f compose.isaac.yaml exec isaac-base bash

source libs/robot_motion_interface/ros/install/setup.bash
source libs/primitives/ros/install/setup.bash

ros2 run robot_motion_interface_ros interface --ros-args \
  -p interface_type:=isaacsim_object \
  -p config_path:=/workspace/libs/robot_motion_interface/config/isaacsim_config.yaml
```

2) Start the UI backend with the same env vars from the smoke test (If you have followed the root README to setup the container and start up the application, skip):
(in another terminal, also inside `isaac-base`):
```bash
sudo docker compose -f compose.isaac.yaml exec isaac-base bash

source libs/robot_motion_interface/ros/install/setup.bash
source libs/primitives/ros/install/setup.bash

# export PYTHONPATH="${PYTHONPATH}:app/ui_backend/src:libs/planning/planning_py/src:libs/sensor_interface/sensor_interface_py/src"

# DEXTERITY_SCENE_SOURCE=yolo \
# DEXTERITY_SCENE_STRICT=1 \
# DEXTERITY_CAMERA=realsense \
# DEXTERITY_CAMERA_CONFIG=libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/realsense_config.yaml \
# DEXTERITY_CAMERA_TRANSFORM=libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/table_world_transform.yaml \
uvicorn ui_backend.api:app --reload
```

3) Spawn objects (host or container; `network_mode: host` makes `localhost` work):
```bash
curl -X POST http://localhost:8000/api/spawn_objects
```

You should see the bowl and cup at the localized poses.

## 5) Accuracy Check (70% within ~2 cm)
Place bowl/cup at known taped XY positions (meters) and run this 20 times.

```bash
export PYTHONPATH="${PYTHONPATH}:app/ui_backend/src:libs/planning/planning_py/src:libs/sensor_interface/sensor_interface_py/src"

DEXTERITY_SCENE_SOURCE=yolo \
DEXTERITY_SCENE_STRICT=1 \
DEXTERITY_CAMERA=realsense \
DEXTERITY_CAMERA_ALIGN=none \
DEXTERITY_CAMERA_CONFIG=libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/realsense_config.yaml \
DEXTERITY_CAMERA_TRANSFORM=libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/table_world_transform.yaml \
python - <<'PY'
import time
import numpy as np
from ui_backend.utils.helpers import get_current_scene

# Update with your taped ground-truth XY (meters)
gt = {
    "cup":  np.array([0.0, 0.0]),
    "bowl": np.array([0.203, 0.0]),
}

N = 20
hits = 0
for _ in range(N):
    scene = get_current_scene()
    by_name = {o["name"]: np.array(o["pose"][:2], dtype=float) for o in scene}
    ok = True
    for name, gt_xy in gt.items():
        est = by_name.get(name)
        if est is None or np.linalg.norm(est - gt_xy) > 0.02:
            ok = False
    hits += int(ok)
    time.sleep(0.25)

print(f"Accuracy: {hits/N:.2f} (target >= 0.70)")
PY
```

## Tuning Knobs (Environment Variables)
These are read by `helpers.py` and can be used to improve accuracy or stability:
- `DEXTERITY_LOCALIZATION_FRAMES` (default 5)
- `DEXTERITY_LOCALIZATION_WARMUP` (default 5)
- `DEXTERITY_LOCALIZATION_TIMEOUT` (default 5.0)
- `DEXTERITY_LOCALIZATION_MIN_DETECTIONS` (default 1)
- `DEXTERITY_MIN_OBJECT_POINTS` (default 30)
- `DEXTERITY_OUTLIER_Z` (default 3.5)
- `DEXTERITY_OUTLIER_MIN_POINTS` (default 30)
- `DEXTERITY_OUTLIER_MIN_KEEP_RATIO` (default 0.3)
- `DEXTERITY_Z_MODE` (default "bottom")
- `DEXTERITY_Z_PERCENTILE` (default 5.0)

## Troubleshooting
- If centroids are offset: re-check `T_world_color` and ensure the world origin is the
  floor at the center of the table.
- If detections are missing: lower `DEXTERITY_YOLO_CONF` or increase `DEXTERITY_LOCALIZATION_FRAMES`.
- If centroids jump: raise `DEXTERITY_OUTLIER_MIN_POINTS` and/or lower `DEXTERITY_OUTLIER_Z`.
- If you see fallback to default poses: set `DEXTERITY_SCENE_STRICT=1` to surface errors.
