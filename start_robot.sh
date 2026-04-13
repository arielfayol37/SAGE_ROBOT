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

# 1) Valpo KB Server
tmux new-window -t "$SESSION" -n "Valpo KB" "bash -lc '
  cd $HOME/Desktop/SAGE_ROBOT/knowledge_base
  docker compose up -d || { echo Valpo KB server failed; sleep 5; }
  exec bash
'"

# 2) Camera
tmux new-window -t "$SESSION" -n "Camera" "bash -lc '
  source $ROS_SETUP || true
  ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:=[480,270] || { echo Camera failed; sleep 5; }
  exec bash
'"

# 3) Video Server
tmux new-window -t "$SESSION" -n "Video Server" "bash -lc '
  source $ROS_SETUP || true
  ros2 run web_video_server web_video_server || { echo web_video_server failed; sleep 5; }
  exec bash
'"

# 4) Web Bridge
tmux new-window -t "$SESSION" -n "Web Bridge" "bash -lc '
  source $ROS_SETUP || true
  source $WS_SETUP || true
  ros2 run web_teleop_bridge control_bridge || { echo control_bridge failed; sleep 5; }
  exec bash
'"

# 5) Teleop Inteface :8001 
tmux new-window -t "$SESSION" -n "Teleop Web" "bash -lc '
  cd $HOME/Desktop/SAGE_ROBOT/interface/teleop_interface
  /usr/bin/python3 -m http.server 8001 || { echo teleop interface failed; sleep 5; }
  exec bash
'"

# 6) Status Interface: 8080
tmux new-window -t "$SESSION" -n "Status Web" "bash -lc '
  cd $HOME/Desktop/SAGE_ROBOT/interface/status_interface
  npx vite || { echo status interface failed; sleep 5; }
  exec bash
'"

# 7) Serial Bridge
tmux new-window -t "$SESSION" -n "Serial Bridge" "bash -lc '
  source $ROS_SETUP || true
  source $WS_SETUP || true
  source \$HOME/Desktop/SAGE_ROBOT/.venv/bin/activate
  which python3
  ros2 run web_teleop_bridge serial_bridge_without_imu || { echo serial_bridge failed; sleep 5; }
  exec bash
'"

# 8) Lidar Scan publisher
tmux new-window -t "$SESSION" -n "Scan Publisher" "bash -lc '
  source $ROS_SETUP || true
  source $WS_SETUP || true
  ros2 launch sllidar_ros2 sllidar_a1_launch.py frame_id:=lidar_link || { echo sllidar /scan publisher failed; sleep 5; }
  exec bash
'"

# 9) Speech
tmux new-window -t "$SESSION" -n "Speech" "bash -lc '
  source $ROS_SETUP || true
  source $WS_SETUP || true
  source \$HOME/Desktop/SAGE_ROBOT/.venv/bin/activate
  cd \$HOME/Desktop/SAGE_ROBOT/speech
  \$HOME/Desktop/SAGE_ROBOT/.venv/bin/python main.py \
    || { echo speech failed; sleep 5; }
  exec bash
'"

# 10) Map Publisher (plus localization)
tmux new-window -t "$SESSION" -n "AMCL" "bash -lc '
  source $ROS_SETUP || true
  source $WS_SETUP || true
  cd $HOME/Desktop/SAGE_ROBOT
  ros2 launch nav2_bringup localization_launch.py map:=/home/agi/Desktop/SAGE_ROBOT/maps/new_save_map.yaml \
  params_file:=/home/agi/Desktop/SAGE_ROBOT/config/nav2_params.yaml || { echo localization + /map publisher failed; sleep 5; }
  exec bash
'"

# 11) Direction of arrival server
tmux new-window -t "$SESSION" -n "DOA" "bash -lc '
  source \$HOME/Desktop/SAGE_ROBOT/.venv/bin/activate
  cd $HOME/Desktop/SAGE_ROBOT/speech
  python doa_server.py || { echo DOA server failed; sleep 5; }
  exec bash
'"

# 12) Nav2
tmux new-window -t "$SESSION" -n "Nav2" "bash -lc '
  source $ROS_SETUP || true
  source $WS_SETUP || true
  ros2 launch nav2_bringup navigation_launch.py params_file:=/home/agi/Desktop/SAGE_ROBOT/config/nav2_params.yaml use_sim_time:=false map_subscribe_transient_local:=true || { echo nav2 failed; sleep 5; }
  exec bash
'"

