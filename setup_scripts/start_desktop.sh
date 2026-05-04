#!/bin/bash
# Opens a tmux session with 4 panes, sets up ROS, and leaves you at a prompt.
#
# Pane layout:
#   ┌──────────────┬──────────────┐
#   │    isaac     │   backend    │
#   ├──────────────┼──────────────┤
#   │   frontend   │     misc     │
#   └──────────────┴──────────────┘

set -e

DIR="$(cd "$(dirname "$0")/.." && pwd)"
SESSION="dexterity"

xhost +local:

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -c "$DIR"        # pane 0.0 (top-left:    isaac)
tmux split-window -v -t "$SESSION:0.0" -c "$DIR"   # pane 0.1 (top-right:   backend)
tmux split-window -h -t "$SESSION:0.0" -c "$DIR"   # pane 0.2 (bottom-left: frontend)
tmux split-window -h -t "$SESSION:0.2" -c "$DIR"   # pane 0.3 (bottom-right: misc)

# Isaac pane (top-left): start container, build ROS, source
tmux send-keys -t "$SESSION:0.0" \
  "docker compose -f docker/compose.isaac.yaml build && docker compose -f docker/compose.isaac.yaml run --rm isaac-base" Enter
tmux send-keys -t "$SESSION:0.0" \
  "source /workspace/scripts/docker_isaac.sh" Enter

# Backend pane (top-right): wait for container, exec in, source ROS
tmux send-keys -t "$SESSION:0.1" \
  "until docker ps | grep -q isaac-base; do sleep 2; done && docker compose -f docker/compose.isaac.yaml exec isaac-base bash" Enter
tmux send-keys -t "$SESSION:0.1" \
  "until test -f /workspace/libs/primitives/ros/install/setup.bash; do sleep 2; done && source /workspace/scripts/docker_backend.sh" Enter

# Frontend pane (bottom-left): build and serve frontend
tmux send-keys -t "$SESSION:0.2" \
  "$DIR/scripts/start_frontend.sh" Enter

tmux select-pane -t "$SESSION:0.0"
tmux attach-session -t "$SESSION"
