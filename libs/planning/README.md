# Dexterity Planning

This module provides LLM-based task planning for the dexterity interface.  
It uses a GPT model to convert natural language tasks into structured JSON primitive plans,  
based on a catalog of low- and mid-level manipulation primitives.

---

## Requirements
* [OPTIONAL] If you are running the perception examples, follow the instructions in the `sensor_interface` package to setup the cameras (either realsense or kinect).

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

## Running Examples

### LLM primitive breakdwon

Show an example of breaking down a high level task into primitives:

```bash
# Run from project root
python -m planning.examples.primitive_breakdwon
```


### MOVE_TRANSPORT Preconditions

This task verifies that the planner correctly loads and respects preconditions for the
`MOVE_TRANSPORT` primitive using a YAML configuration file.  
It generates a structured JSON output for **3 scenes × 4 precondition sets = 12 cases**.


```bash
# Run from project root
python -m planning.examples.test_preconditions
```


### YOLO Kitchen Scene Overlay Visualization

This visual test overlays segmentation masks and labels over example kitchen images.

```bash
source venv-dex/bin/activate
pip install -e libs/planning/planning_py

python -m planning.examples.rgb_yolo_overlay_only \
  --config libs/planning/planning_py/src/planning/config/kitchen_images.yaml
```

NOTE: Some kitchen images (e.g., `Kitchen_10.jpg`) are encoded as AVIF (ISO Media).
Linux systems require `libheif` to decode them which you may need to install:
```bash
sudo apt update
sudo apt install libheif-dev
```


### YOLO RGB-D Perception 
Outputs kitchen scene depth image, segmentation, and point cloud
```bash
python3 -m planning.examples.rgbd_yolo_example --config libs/planning/planning_py/src/planning/config/kitchen_example.yaml --output yolo_pointcloud.png
```  

### YOLO RGB-D Streaming 
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
