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

## 1) Quick Visual Sanity: YOLO RGB-D Stream
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

## 2) Measure and Set the Camera-to-World Transform
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

Calibration Procudure (2-3 points):
- Set the `T_world_color` to identity
- Put the cup at (0, 0),(x, 0), (0, y), (x, y) each time and run the script below and record the results (the coordinate should be origin on the floor of the table center)
```bash
export PYTHONPATH="${PYTHONPATH}:app/ui_backend/src:libs/planning/planning_py/src:libs/sensor_interface/sensor_interface_py/src"
DEXTERITY_SCENE_SOURCE=yolo \
DEXTERITY_CAMERA=realsense \
DEXTERITY_CAMERA_ALIGN=none \
DEXTERITY_CAMERA_CONFIG=libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/realsense_config.yaml \
DEXTERITY_CAMERA_TRANSFORM=libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/table_world_transform.yaml \
python - <<'PY'
import json
from ui_backend.utils import helpers
perception_settings = helpers._localization_settings()
camera = helpers._init_camera(perception_settings)
yolo = helpers._init_yolo(camera, perception_settings)
print(json.dumps(helpers._localize_scene(camera, yolo, perception_settings), indent=2))
PY
# real-world position at (0, 0)
# {
#   "name": "cup",
#   "description": "Small cup. Height: 0.08 (m). Use this grasp pose: [0.2, 0.11, 1, 0, -0.818, 0.574, 0]",
#   "pose": [
#     -0.025334328413009644,
#     -0.06962248682975769,
#     0.6223821043968201,
#     0.0,
#     0.0,
#     0.0,
#     1.0
#   ]
# }
# real-world position at (0.1, 0)
# {
#   "name": "cup",
#   "description": "Small cup. Height: 0.08 (m). Use this grasp pose: [0.2, 0.11, 1, 0, -0.818, 0.574, 0]",
#   "pose": [
#     -0.11780847609043121,
#     -0.06689731031656265,
#     0.6233230829238892,
#     0.0,
#     0.0,
#     0.0,
#     1.0
#   ]
# }
# real-world position at (0, 0.1)
# {
#   "name": "cup",
#   "description": "Small cup. Height: 0.08 (m). Use this grasp pose: [0.2, 0.11, 1, 0, -0.818, 0.574, 0]",
#   "pose": [
#     -0.027526579797267914,
#     -0.013532162643969059,
#     0.5381515026092529,
#     0.0,
#     0.0,
#     0.0,
#     1.0
#   ]
# }
```
- Before collecting points, set `T_world_color` to identity in
  `libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/table_world_transform.yaml`.
- Run the following script with results as the input to compute the transform matrix
```bash
python - <<'PY'
import numpy as np

# Paste your measured camera-frame points here (output from _localize_scene)
# Example format: [x, y, z] in meters
camera_pts = np.array([
    [-0.025334328413009644, -0.06962248682975769, 0.6223821043968201],
    [-0.11780847609043121, -0.06689731031656265, 0.6233230829238892],
    [-0.027526579797267914, -0.013532162643969059, 0.5381515026092529],
], dtype=float)

# Known world XY positions (meters) for each point above
world_xy = np.array([
    [0.0, 0.0],
    [0.1, 0.0],
    [0.0, 0.1],
], dtype=float)

# Set world Z for the cup center.
# If origin is floor, z = table_height + 0.5 * cup_height.
table_height = 0.94   # <-- measure this (floor -> tabletop)
cup_height = 0.08
z0 = table_height + 0.5 * cup_height

world_pts = np.column_stack([world_xy, np.full(len(world_xy), z0)])

def rigid_transform(P, Q):
    Pc = P.mean(axis=0)
    Qc = Q.mean(axis=0)
    X = P - Pc
    Y = Q - Qc
    H = X.T @ Y
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = Qc - R @ Pc
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

T = rigid_transform(camera_pts, world_pts)

print("T_world_color:")
for row in T:
    print("  - [" + ", ".join(f"{v:.9f}" for v in row) + "]")

# Quick fit check (RMSE in meters)
res = (camera_pts @ T[:3, :3].T + T[:3, 3]) - world_pts
rmse = np.sqrt((res ** 2).mean())
print(f"RMSE: {rmse:.4f} m")
PY
# Calibration Notes (2026-02-11):
# Camera height above table surface: 0.35 m
# Camera distance to nearest table edge: 0.23 m
# Computed T_world_color (table_height = 0.94 m):
#   - [-0.999089216, 0.008381457, 0.041838848, -0.047963599]
#   - [-0.030184535, 0.554228009, -0.831817412, 0.553951493]
#   - [-0.030160103, -0.832322693, -0.553470237, 1.265757509]
#   - [0.000000000, 0.000000000, 0.000000000, 1.000000000]
# Fit RMSE: 0.0021 m
# If your table_height differs, add (table_height - 0.94) to T_world_color[2][3].
# Implied camera height above table from these points: 0.3258 m (re-check if you expect 0.35 m).
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
from ui_backend.utils import helpers
import json
perception_settings = helpers._localization_settings()
camera = helpers._init_camera(perception_settings)
yolo = helpers._init_yolo(camera, perception_settings)
print(json.dumps(helpers.get_current_scene(camera, yolo, perception_settings), indent=2))
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
from ui_backend.utils import helpers
perception_settings = helpers._localization_settings()
camera = helpers._init_camera(perception_settings)
yolo = helpers._init_yolo(camera, perception_settings)

# Update with your taped ground-truth XY (meters)
gt = {
    "cup":  np.array([0.0, 0.0]),
    "bowl": np.array([0.2, 0.0]),
}

N = 20
hits = 0
for _ in range(N):
    scene = helpers.get_current_scene(camera, yolo, perception_settings)
    by_name = {o["name"]: np.array(o["pose"][:2], dtype=float) for o in scene}
    ok = True
    for name, gt_xy in gt.items():
        est = by_name.get(name)
        print(name, est, np.linalg.norm(est - gt_xy))
        if est is None or np.linalg.norm(est - gt_xy) > 0.05:
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
