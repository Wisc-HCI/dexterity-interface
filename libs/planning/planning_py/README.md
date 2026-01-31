# Dexterity Planning

This module provides LLM-based task planning for the dexterity interface.  
It uses a GPT model to convert natural language tasks into structured JSON primitive plans,  
based on a catalog of low- and mid-level manipulation primitives.

---

## Setup

```bash
# Create and activate virtual environment
python3 -m venv venv-dex
source venv-dex/bin/activate     # Windows: venv-dex\Scripts\activate

# Install planning package and dependencies
pip install -e libs/planning/planning_py

# Add a .env file with the following format
# (place it in libs/planning)
# Example:
OPENAI_API_KEY=your_openai_api_key_here
OPENAI_MODEL=gpt-5-nano

# Run the example to test the planner
python -m planning.examples.primitive_breakdown
```

---

If using camera, make sure you install the dependencies in  /libs/sensor_interface/README.md

## Test MOVE_TRANSPORT Preconditions

This task verifies that the planner correctly loads and respects preconditions for the
`MOVE_TRANSPORT` primitive using a YAML configuration file.  
It generates a structured JSON output for **3 scenes × 4 precondition sets = 12 cases**.

### Run

```bash
# Run from project root
python -m planning.examples.test_preconditions
```

---

## YOLO Kitchen Scene Overlay Visualization

This visual test overlays segmentation masks and labels over example kitchen images.

```bash
source venv-dex/bin/activate
pip install -e libs/planning/planning_py

python -m planning.examples.rgb_yolo_overlay_only \
  --config libs/planning/planning_py/src/planning/config/kitchen_images.yaml
```


### YOLO RGB-D Perception Demo
Outputs kitchen scene depth image, segmentation, and point cloud
```bash
python3 -m planning.examples.rgbd_yolo_example --config libs/planning/planning_py/src/planning/config/kitchen_example.yaml --output yolo_pointcloud.png
```  

### YOLO RGB-D Streaming Demo
Live segmented RGB/Depth, point clouds, and centroids using Kinect or RealSense (press `q` to quit):
- RealSense: `python3 -m planning.examples.rgbd_yolo_stream --camera realsense --camera-config libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/realsense_config.yaml --model libs/planning/planning_py/src/planning/yolo11n-seg.pt`
- Kinect:
```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export QT_QPA_PLATFORM=xcb
python3 -m planning.examples.rgbd_yolo_stream --camera kinect --align color \
  --camera-config libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/kinect_config.yaml \
  --model libs/planning/planning_py/src/planning/yolo11n-seg.pt
```

Make sure the corresponding driver is installed (`pyrealsense2` for RealSense or `pyk4a` for Kinect) along with `ultralytics`, `opencv-python`, and `matplotlib`. Point the camera at the kitchen objects to quickly verify segmentation accuracy, point cloud alignment, and centroid locations.


### YOLO RGB-D PointCloud output
Output stream of pointcloud and saves to ./pointcloud_output/.
```bash
pip install open3d

# Kinect example
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export QT_QPA_PLATFORM=xcb
python3 -m planning.examples.rgbd_pointcloud_output --camera kinect --align color \
  --camera-config libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config/kinect_config.yaml \
  --model libs/planning/planning_py/src/planning/yolo11n-seg.pt
```  




### Linux Dependency for AVIF Image Support
Some kitchen images (e.g., `Kitchen_10.jpg`) are encoded as AVIF (ISO Media).
Linux systems require `libheif` to decode them:
```bash
sudo apt update
sudo apt install libheif-dev
```
This is required for loading AVIF-format kitchen images on Linux.