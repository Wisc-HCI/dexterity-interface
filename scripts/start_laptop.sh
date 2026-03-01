#!/bin/bash
# Launches the laptop workspace in a single terminal.
# Starts the ROS docker container, builds packages.

set -e

DIR="$(cd "$(dirname "$0")/.." && pwd)"

xhost +local:


docker compose -f "$DIR/compose.ros.yaml" run --rm ros-base /workspace/scripts/docker_control.sh