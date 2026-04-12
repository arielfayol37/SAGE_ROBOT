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

  DESCRIPTION=\$(xacro \$HOME/Desktop/SAGE_ROBOT/description/sagewithoutimu.urdf.xacro)

  ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:=\"\$DESCRIPTION\" \
  || { echo Robot State Publisher failed; sleep 5; }

  exec bash
'"

# 1) Camera
tmux new-window -t "$SESSION" -n "Camera" "bash -lc '
  source $ROS_SETUP || true
  ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:=[480,270] || { echo Camera failed; sleep 5; }
  exec bash
'"

# 2) Video Server
tmux new-window -t "$SESSION" -n "Video Server" "bash -lc '
  source $ROS_SETUP || true
  ros2 run web_video_server web_video_server || { echo web_video_server failed; sleep 5; }
  exec bash
'"

# 3) Web Bridge
tmux new-window -t "$SESSION" -n "Web Bridge" "bash -lc '
  source $ROS_SETUP || true
  source $WS_SETUP || true
  ros2 run web_teleop_bridge control_bridge || { echo control_bridge failed; sleep 5; }
  exec bash
'"

# 4) Teleop Inteface :8001 
tmux new-window -t "$SESSION" -n "Teleop Web" "bash -lc '
  cd $HOME/Desktop/SAGE_ROBOT/interface/teleop_interface
  /usr/bin/python3 -m http.server 8001 || { echo teleop interface failed; sleep 5; }
  exec bash
'"

# 5) Serial Bridge
tmux new-window -t "$SESSION" -n "Serial Bridge" "bash -lc '
  source $ROS_SETUP || true
  source $WS_SETUP || true
  source \$HOME/Desktop/SAGE_ROBOT/.venv/bin/activate
  which python3
  ros2 run web_teleop_bridge serial_bridge_without_imu || { echo serial_bridge failed; sleep 5; }
  exec bash
'"

# 6) Lidar Scan publisher
tmux new-window -t "$SESSION" -n "Scan Publisher" "bash -lc '
  source $ROS_SETUP || true
  source $WS_SETUP || true
  ros2 launch sllidar_ros2 sllidar_a1_launch.py frame_id:=lidar_link || { echo sllidar /scan publisher failed; sleep 5; }
  exec bash
'"

# 7) Map Publisher (plus localization)
tmux new-window -t "$SESSION" -n "SLAM" "bash -lc '
  source $ROS_SETUP || true
  source $WS_SETUP || true
  ros2 launch slam_toolbox online_async_launch.py || { echo localization + /map publisher failed; sleep 5; }
  exec bash
'"

# 8) Nav2
tmux new-window -t "$SESSION" -n "Nav2" "bash -lc '
  source $ROS_SETUP || true
  source $WS_SETUP || true
  ros2 launch nav2_bringup navigation_launch.py params_file:=/home/agi/Desktop/SAGE_ROBOT/config/nav2_params.yaml use_sim_time:=false || { echo nav2 failed; sleep 5; }
  exec bash
'"

