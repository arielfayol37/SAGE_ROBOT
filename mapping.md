agi@SAGE:~$ ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/Desktop/SAGE_ROBOT/description/sage.urdf.xacro)"


agi@SAGE:~/Desktop/SAGE_ROBOT/ros2_ws$ source install/local_setup.bash
agi@SAGE:~/Desktop/SAGE_ROBOT/ros2_ws$ ros2 launch sllidar_ros2 sllidar_a1_launch.py frame_id:=lidar_link

agi@SAGE:~$ ros2 launch slam_toolbox online_async_launch.py
