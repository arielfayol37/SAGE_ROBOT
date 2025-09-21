#!/bin/bash
set -e

SESSION="sage"
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"
tmux new-session -d -s "$SESSION"

tmux rename-window -t "$SESSION:0" "Camera"
tmux send-keys   -t "$SESSION:0" 'ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:=[480,270]' C-m

tmux new-window  -t "$SESSION" -n "Video Server"
tmux send-keys   -t "$SESSION:1" 'ros2 run web_video_server web_video_server' C-m

tmux new-window  -t "$SESSION" -n "Static Files"
tmux send-keys   -t "$SESSION:2" 'cd ~/Desktop/SAGE_ROBOT/signaling/static && sudo /usr/bin/python3 -m http.server 80' C-m

tmux new-window  -t "$SESSION" -n "Signaling"
tmux send-keys   -t "$SESSION:3" 'cd ~/Desktop/SAGE_ROBOT/signaling && source .signaling_venv/bin/activate && python signaling_server.py' C-m

tmux new-window  -t "$SESSION" -n "Teleop Bridge"
tmux send-keys   -t "$SESSION:4" 'cd ~/Desktop/SAGE_ROBOT/ros2_ws && source install/local_setup.bash && ros2 run web_teleop_bridge control_bridge' C-m

tmux new-window  -t "$SESSION" -n "Serial Bridge"
tmux send-keys   -t "$SESSION:5" 'cd ~/Desktop/SAGE_ROBOT/ros2_ws && source install/local_setup.bash && ros2 run web_teleop_bridge serial_bridge' C-m

tmux new-window  -t "$SESSION" -n "Turtlesim"
tmux send-keys   -t "$SESSION:6" 'ros2 run turtlesim turtlesim_node' C-m

tmux attach -t "$SESSION"
