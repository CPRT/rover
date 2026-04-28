ros2 launch navigation slam.launch.py > slam_log.txt 2>&1 &
ros2 run localization repub_odom > rpub_odom.txt 2>&1 &
ros2 launch navigation nav2.launch.py > nav2.log 2>&1 &
ros2 launch video_streaming video_streaming.launch.py > video.log 2>&1 &

ros2 launch bringup control.launch.py use_arm:=false

python3 send_gps_command.py --file p7_feb22_2026_aruco.yaml 
ros2 launch nav_commanders incremental_gps_commander.launch.py

ros2 param set /detect_node detection_type "ARUCO"