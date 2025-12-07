std_msgs/Header header

# Board identification
string board_id  # e.g., "Tag6-Tag7"
int64[] marker_ids  # IDs of the markers in this board

# Pose of the board frame relative to the camera frame
geometry_msgs/Pose pose

# Corner positions in 3D camera frame for debugging/visualization
geometry_msgs/Point[] corners_3d
