#!/bin/bash
# Launches the laptop workspace in a single terminal.
# Starts the ROS docker container, builds packages.

set -e

BUILD=true
for arg in "$@"; do
  [[ "$arg" == "--no-build" ]] && BUILD=false
done

DIR="$(cd "$(dirname "$0")/.." && pwd)"

xhost +local:

if $BUILD; then
  docker compose -f "$DIR/docker/compose.ros.yaml" build
fi
docker compose -f "$DIR/docker/compose.ros.yaml" run --rm ros-base /workspace/setup_scripts/docker_control.sh