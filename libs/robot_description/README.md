# Robot Description


## Notes
TODO: Make this into python package OR move functionality to isaacsim???

If you want to convert xacro to urdf (for isaacsim, for example), you can do the following:
#### Installation
```bash
pip install xacro
```
#### Running
```bash
python3 -m xacro robot.urdf.xacro -o robot.urdf
```

For example:
```bash
xacro panda/panda_with_kinect.urdf.xacro -o panda/panda_with_kinect.urdf
```