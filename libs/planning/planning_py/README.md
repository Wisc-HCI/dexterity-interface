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

### YOLO RGB-D PointCloud output
Outputs kitchen scene depth image, segmentation, and point cloud
```bash
python3 -m planning.examples.rgbd_yolo_example --config libs/planning/planning_py/src/planning/config/kitchen_example.yaml --output yolo_pointcloud.png
```  




### Linux Dependency for AVIF Image Support
Some kitchen images (e.g., `Kitchen_10.jpg`) are encoded as AVIF (ISO Media).
Linux systems require `libheif` to decode them:
```bash
sudo apt update
sudo apt install libheif-dev
```
This is required for loading AVIF-format kitchen images on Linux.