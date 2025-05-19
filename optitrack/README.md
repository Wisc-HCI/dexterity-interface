# Preparation time
- 30 minutes ~ 1 hour for full setup

# Camera setup
- more cameras the better (refrain from any occlusions); see picture below for setup
![camera-setup1](./readme-images/camera1.png)
![camera-setup2](./readme-images/camera2.png)
- all ethernet cables should be connected to the cameras and to the switch
- if the camera is not picking up, restart software/plug in to different ports on the switch/change ethernet cable; currently 5 cameras setup. the number of each camera shows on the bottom left

# Workspace (worktable) setup and starting capture
## Initialization
- marker on center of the table, dim the brightness of the room (so curtains down) and confirm that the markers are showing in the Motive software (you can adjust exposure per camera)

## Calibration
- in order to only capture the marker: ``left panel > Calibration > Mask``
- ``press empty space (and not camera footage) > left panel > Wanding > Start Wanding``, and then wave the wand so that all cameras accumulate enough samples (around 1000 ~ 2000 , is the rule of thumb), and then click ``Calculate`` to see quality of calibration samples (e.g., ``Exceptional``)
![wand-and-exposure](./readme-images/wand-exposure.png)
- grounding each camera to the physical coordinates: see picture. make sure one side of the triangle is longer than the other side
![grounding1](./readme-images/grounding1.png)
- select the three dots, click on ``Set ground plane``. then the 90 degree angle dot is saved as the (0,0,0) coordinate in the simulated world within the software
![grounding2](./readme-images/grounding2.png)

## (menu on the top) ``Layout > Capture``v
- bottom timeline: ``Live``; press red button to record
- you can label each marker (more camera, less time to label markers); ``Labeling > Manual Labeling`` per frame if needed
![labeling](./readme-images/labeling.png)

# Visualization + Analysis
- save in ``*.c3d`` for visualization and ``*.csv`` for raw data extraction
- specific details TBD