#!/bin/bash
source ~/gstreamer/setupGstreamer.sh
source /opt/ros/humble/setup.bash
source /opt/ros/humble/cprt_setup.bash
sudo enablecan.sh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml