#!/bin/bash

# Default recording directory (matches your example)
REC_DIR="/usr/local/zed/recordings"

# Help Menu
if [[ "$1" == "--help" || "$1" == "-h" ]]; then
    echo "Usage: $0 <filename>"
    echo ""
    echo "Description:"
    echo "  Starts a ZED camera SVO recording via ROS2 service call."
    echo "  Files are saved to: $REC_DIR"
    echo ""
    echo "Arguments:"
    echo "  filename   The name of the output file (e.g., 'mission_test')."
    echo "             The script automatically appends '.svo2' if missing."
    echo ""
    echo "Options:"
    echo "  -h, --help Show this help message and exit."
    exit 0
fi

# Check for argument
if [ -z "$1" ]; then
    echo "Error: No filename provided."
    echo "Try '$0 --help' for usage."
    exit 1
fi

# Construct filename
FILENAME="$1"
# Append extension if user didn't provide it
if [[ "$FILENAME" != *.svo2 ]]; then
    FILENAME="${FILENAME}.svo2"
fi

# Ensure full path
FULL_PATH="${REC_DIR}/${FILENAME}"

echo "Starting ZED recording..."
echo "Saving to: $FULL_PATH"

# Execute Service Call
# compression_mode: 2 (H.265/HEVC - High compression, good quality)
ros2 service call /zed/zed_node/start_svo_rec zed_msgs/srv/StartSvoRec "{
    svo_filename: '$FULL_PATH', 
    compression_mode: 2, 
    bitrate: 0, 
    target_framerate: 0, 
    input_transcode: false
}"