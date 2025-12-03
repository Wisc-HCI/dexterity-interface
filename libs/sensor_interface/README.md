# Sensor Interface Library

This library provides sensor utilities for RGB cameras, depth cameras, and other data sources used in the dexterity-interface project.  
Currently it includes support for Intel RealSense RGB-D cameras via **pyrealsense2**.

---

## Installation
Then install the package in editable mode:
```bash
pip install -e libs/sensor_interface/sensor_interface_py
```

### Running the RealSense Example
Run the RealSense streaming example using module syntax:
```bash
python3 -m sensor_interface.camera.examples.realsense_stream_example
```
This example opens a RealSense pipeline, streams RGB and depth frames, and displays the results using OpenCV.
