#!/bin/bash

# Define the bag name with a timestamp so you don't overwrite previous bags
BAG_NAME="autonomy_debug_bag_$(date +%Y_%m_%d_%H_%M_%S)"

echo "Starting ROS 2 bag recording: $BAG_NAME"
echo "Press Ctrl+C to stop recording."

# Run the ros2 bag record command
ros2 bag record -o "$BAG_NAME" \
    /odometry/filtered/global \
    /global_costmap/costmap \
    /global_costmap/costmap_updates \
    /traversability_map \
    /plan \
    /unsmoothed_plan \
    /goal_pose \
    /goal \
    /zed/zed_node/left/image_rect_color/compressed \
    /zed/zed_node/left/camera_info \
    /tf \
    /tf_static \
    /marker_detected \
    /intended_search_path

# Note: /tf and /tf_static were added (see explanation below)