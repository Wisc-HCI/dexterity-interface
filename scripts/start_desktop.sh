#!/bin/bash
# Opens a tmux session with 3 panes, sets up ROS, and leaves you at a prompt.
#
# Pane layout:
#   ┌─────────────────────────────┐
#   │  isaac  (docker + ros sim)  │
#   ├──────────────┬──────────────┤
#   │   backend    │   frontend   │
#   └──────────────┴──────────────┘

set -e

DIR="$(cd "$(dirname "$0")/.." && pwd)"
SESSION="dexterity"

xhost +local:

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -c "$DIR"

# Split into 3 panes: top, bottom-left, bottom-right
tmux split-window -v -t "$SESSION" -c "$DIR"
tmux split-window -h -t "$SESSION:0.1" -c "$DIR"

# Isaac pane (top): start container, build ROS, source
tmux send-keys -t "$SESSION:0.0" \
  "docker compose -f compose.isaac.yaml run --rm  isaac-base" Enter

# Isaac pane (top): Build ROS, source
tmux send-keys -t "$SESSION:0.0" \
  "source /workspace/scripts/docker_isaac.sh" Enter

# Backend pane (bottom-left): wait for container, 
tmux send-keys -t "$SESSION:0.1" \
  "until docker ps | grep -q isaac-base; do sleep 2; done && docker compose -f compose.isaac.yaml exec isaac-base bash" Enter

# Backend pane (bottom-left): wait ROS workspace to be build, then source ROS
tmux send-keys -t "$SESSION:0.1" \
  "until test -f /workspace/libs/primitives/ros/install/setup.bash; do sleep 2; done && source /workspace/scripts/docker_backend.sh" Enter

# Frontend pane (bottom-right): build frontend
tmux send-keys -t "$SESSION:0.2" \
  "$DIR/scripts/start_frontend.sh" Enter

tmux select-pane -t "$SESSION:0.0"
tmux attach-session -t "$SESSION"
