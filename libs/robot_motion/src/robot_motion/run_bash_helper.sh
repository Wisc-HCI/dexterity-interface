#!/usr/bin/env bash

echo "🏗️  Building packages from libs/..."
colcon build --symlink-install --packages-select robot_description robot_motion

echo "🔧  Sourcing workspace..."
source install/setup.bash

echo "🚀  Launching display.launch.py..."
ros2 launch robot_description display.launch.py

exec bash

# #!/bin/bash
# # ============================================================
# # run_display.sh
# # Rebuilds both workspaces and launches the robot display
# # ============================================================


# # --- Move to robot_description workspace and build ---
# echo "🏗️  Building robot_description..."
# cd ../../../robot_description/
# colcon build --symlink-install
# source install/setup.bash

# # --- Move to robot_motion workspace and build ---
# echo "🤖  Building robot_motion..."
# cd ../robot_motion/src/robot_motion/
# colcon build --symlink-install
# source install/setup.bash

# # --- Launch display ---
# echo "🚀  Launching display.launch.py..."
# ros2 launch robot_description display.launch.py


# exec bash
