#!/usr/bin/env bash
export TERM=xterm-256color
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native

set -Eeuo pipefail

SESSION="sage"

# Clean old session if present
tmux kill-session -t "$SESSION" 2>/dev/null || true

# Adjust these if your paths differ
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$HOME/Desktop/SAGE_ROBOT/ros2_ws/install/local_setup.bash"

# 0) Robot State Publisher
tmux new-session -d -s "$SESSION" -n "RS Publisher" "bash -lc '
  source $ROS_SETUP || true
  source $WS_SETUP || true

  DESCRIPTION=\$(xacro \$HOME/Desktop/SAGE_ROBOT/description/sage.urdf.xacro)

  ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:=\"\$DESCRIPTION\" \
  || { echo Robot State Publisher failed; sleep 5; }

  exec bash
'"


# 0) Camera
tmux new-window -t "$SESSION" -n "Camera" "bash -lc '
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

# 7) Lidar Scan publisher
tmux new-window -t "$SESSION" -n "Scan Publisher" "bash -lc '
  source $ROS_SETUP || true
  source $WS_SETUP || true
  ros2 launch sllidar_ros2 sllidar_a1_launch.py frame_id:=lidar_link || { echo sllidar /scan publisher failed; sleep 5; }
  exec bash
'"

# Speech
tmux new-window -t "$SESSION" -n "Speech" "bash -lc '
  cd $HOME/Desktop/SAGE_ROBOT
  source .venv/bin/activate
  cd $HOME/Desktop/SAGE_ROBOT/speech
  python llm_streaming.py || { echo speech failed; sleep 5; }
  exec bash
'"

# 8) Map Publisher (plus localization)
tmux new-window -t "$SESSION" -n "AMCL" "bash -lc '
  source $ROS_SETUP || true
  source $WS_SETUP || true
  cd $HOME/Desktop/SAGE_ROBOT
  ros2 launch nav2_bringup localization_launch.py map:=Good_Gelly_Save_MAP.yaml \
  params_file:=/home/agi/Desktop/SAGE_ROBOT/config/nav2_params.yaml || { echo localization + /map publisher failed; sleep 5; }
  exec bash
'"

# 9) Nav2
tmux new-window -t "$SESSION" -n "Nav2" "bash -lc '
  source $ROS_SETUP || true
  source $WS_SETUP || true
  ros2 launch nav2_bringup navigation_launch.py params_file:=/home/agi/Desktop/SAGE_ROBOT/config/nav2_params.yaml use_sim_time:=false map_subscribe_transient_local:=true || { echo nav2 failed; sleep 5; }
  exec bash
'"

