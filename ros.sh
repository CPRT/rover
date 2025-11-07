
SCRIPT_PATH="${BASH_SOURCE[0]}"
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
source /opt/ros/humble/setup.bash
source $SCRIPT_DIR/install/setup.bash
