# Robot Description


## Notes
TODO: Make this into python package OR move functionality to isaacsim???

If you want to convert xacro to urdf (for isaacsim, for example), you can do the following:
#### Installation
```bash
pip install xacro
```
#### Running
Make sure you are in this root director (`robot_description`).
```bash
python3 -m xacro robot.urdf.xacro -o composities/urdfsrobot.urdf
```

For example:
```bash
mkdir -p $(pwd)/composites/tmp
xacro composites/panda_with_kinect.urdf.xacro \
  kinect_file_prefix:="$(pwd)/kinect" \
  panda_file_prefix:="$(pwd)/panda" \
  -o $(pwd)/composites/tmp/panda_with_kinect.urdf
```