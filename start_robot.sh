#!/usr/bin/env bash
export TERM=xterm-256color

set -Eeuo pipefail

SESSION="sage"

# Clean old session if present
tmux kill-session -t "$SESSION" 2>/dev/null || true

# Adjust these if your paths differ
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$HOME/Desktop/SAGE_ROBOT/ros2_ws/install/local_setup.bash"

# 0) Camera
tmux new-session -d -s "$SESSION" -n "Camera" "bash -lc '
  source $ROS_SETUP || true
  ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:=[480,270] || { echo Camera failed; sleep 5; }
  exec bash
'"

# 1) Video Server
tmux new-window -t "$SESSION" -n "Video Server" "bash -lc '
  source $ROS_SETUP || true
  ros2 run web_video_server web_video_server || { echo web_video_server failed; sleep 5; }
  exec bash
'"

# 2) Static Files on :8001 
tmux new-window -t "$SESSION" -n "Static Files" "bash -lc '
  cd $HOME/Desktop/SAGE_ROBOT/signaling/static
  /usr/bin/python3 -m http.server 8001 || { echo http.server failed; sleep 5; }
  exec bash
'"

# 3) Signaling server (venv)
tmux new-window -t "$SESSION" -n "Signaling" "bash -lc '
  cd $HOME/Desktop/SAGE_ROBOT/signaling
  source .signaling_venv/bin/activate
  python signaling_server.py || { echo signaling_server failed; sleep 5; }
  exec bash
'"

# 4) Teleop Bridge
tmux new-window -t "$SESSION" -n "Teleop Bridge" "bash -lc '
  source $ROS_SETUP || true
  source $WS_SETUP || true
  ros2 run web_teleop_bridge control_bridge || { echo control_bridge failed; sleep 5; }
  exec bash
'"

# 5) Serial Bridge
tmux new-window -t "$SESSION" -n "Serial Bridge" "bash -lc '
  source $ROS_SETUP || true
  source $WS_SETUP || true
  ros2 run web_teleop_bridge serial_bridge || { echo serial_bridge failed; sleep 5; }
  exec bash
'"

# 6) Turtlesim (optional)
# tmux new-window -t "$SESSION" -n "Turtlesim" "bash -lc '
#  source $ROS_SETUP || true
#  ros2 run turtlesim turtlesim_node || { echo turtlesim failed; sleep 5; }
#  exec bash
# '"

