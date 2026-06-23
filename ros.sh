ROS_DISTRO="${ROS_DISTRO:-jazzy}"

SCRIPT_PATH="${BASH_SOURCE[0]}"
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
source /opt/ros/${ROS_DISTRO}/setup.bash
source $SCRIPT_DIR/install/setup.bash
source $SCRIPT_DIR/install/local_setup.bash