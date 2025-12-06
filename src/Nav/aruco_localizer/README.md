# ArUco Localizer Package

Complete ROS2 package for camera publishing and ArUco board-based localization.

## Overview

This package provides:
1. **Camera Publisher Node**: Publishes camera images and calibration info from USB cameras
2. **ArUco Board Detector Node**: Detects pairs of ArUco markers as boards for precise pose estimation

## Quick Start

### Build the Package

```bash
cd /workspaces/CPRT/rover
colcon build --packages-select interfaces aruco_localizer
source install/setup.bash
```

### Run Complete Pipeline

```bash
# Launch camera + ArUco detector together
ros2 launch aruco_localizer aruco_camera_pipeline.launch.py
```

### Run Individual Nodes

```bash
# Just camera publisher
ros2 launch aruco_localizer camera_publisher.launch.py

# Just ArUco detector (expects camera already running)
ros2 launch aruco_localizer aruco_board_detector.launch.py
```

## Nodes

### 1. camera_publisher_node
- Opens USB camera with OpenCV V4L2
- Publishes images to `/camera/image_raw`
- Publishes camera_info to `/camera/camera_info`
- Reads calibration from YAML files in `config/camera_intrinsics/`

[Full Documentation](README_CAMERA_PUBLISHER.md)

### 2. aruco_board_detector_node
- Detects ArUco marker pairs configured as boards
- Uses `cv2.aruco.estimatePoseBoard` for accurate pose estimation
- Publishes board poses to `/aruco_boards`
- Publishes debug visualization to `/aruco_debug`
- Reads board configs from `config/aruco_boards/`

[Full Documentation](README_BOARD_DETECTOR.md)

## Configuration

### Camera Calibration
Place camera calibration YAML files in:
```
config/camera_intrinsics/
├── ErikKlarityCam.yaml
└── YourCamera.yaml
```

### ArUco Board Definitions
Place board configuration YAML files in:
```
config/aruco_boards/
├── camera-0_ArucoBoard_Tag6-Tag7.yaml
└── YourBoard.yaml
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | sensor_msgs/Image | Raw camera images |
| `/camera/camera_info` | sensor_msgs/CameraInfo | Camera calibration |
| `/aruco_boards` | interfaces/ArucoBoard | Detected board poses |
| `/aruco_debug` | sensor_msgs/Image | Visualization with markers and axes |
| `/aruco_markers_viz` | visualization_msgs/MarkerArray | 3D markers for RViz2 visualization |

## Common Commands

```bash
# View camera stream
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# View debug output with detected markers
ros2 run rqt_image_view rqt_image_view /aruco_debug

# Monitor detected boards
ros2 topic echo /aruco_boards

# Check camera info
ros2 topic echo /camera/camera_info

# View 3D markers in RViz2
rviz2
# Then add MarkerArray display for /aruco_markers_viz topic

# List available cameras
ls -l /dev/video*
```

## Launch File Parameters

All launch files now primarily use config files for parameters, with only essential overrides exposed.

### aruco_camera_pipeline.launch.py (Complete Pipeline)
```bash
# Run with defaults (uses config files)
ros2 launch aruco_localizer aruco_camera_pipeline.launch.py

# Override specific config files
ros2 launch aruco_localizer aruco_camera_pipeline.launch.py \
    calibration_file:=/path/to/calibration.yaml \
    board_config_dir:=/path/to/boards
```

**Arguments:**
- `camera_params_file`: Path to camera parameters YAML (default: config/camera_params.yaml)
- `aruco_params_file`: Path to ArUco parameters YAML (default: config/aruco_board_params.yaml)
- `calibration_file`: Path to camera calibration YAML (default: config/camera_intrinsics/ErikKlarityCam.yaml)
- `board_config_dir`: Directory with board configs (default: config/aruco_boards)

### camera_publisher.launch.py
```bash
# Run with defaults
ros2 launch aruco_localizer camera_publisher.launch.py

# Override calibration file
ros2 launch aruco_localizer camera_publisher.launch.py \
    calibration_file:=/path/to/my_camera.yaml
```

**Arguments:**
- `params_file`: Path to camera parameters YAML (default: config/camera_params.yaml)
- `calibration_file`: Path to camera calibration YAML
- `camera_device`: Override camera device (empty = use params file)

### aruco_board_detector.launch.py
```bash
# Run with defaults
ros2 launch aruco_localizer aruco_board_detector.launch.py

# Override board directory
ros2 launch aruco_localizer aruco_board_detector.launch.py \
    board_config_dir:=/path/to/my_boards
```

**Arguments:**
- `params_file`: Path to ArUco parameters YAML (default: config/aruco_board_params.yaml)
- `board_config_dir`: Directory with board configs (default: config/aruco_boards)
- `image_topic`: Override image topic (empty = use params file)
- `camera_info_topic`: Override camera info topic (empty = use params file)

> **Note:** To change most parameters (resolution, frame rate, topics, etc.), edit the config files in `config/`. Launch arguments are only for quick file path overrides.

## Dependencies

- ROS2 Humble or later
- OpenCV 4.12+ with ArUco module
- cv_bridge
- tf_transformations
- PyYAML
- NumPy

## Package Structure

```
aruco_localizer/
├── aruco_localizer/
│   ├── aruco_board_detector_node.py
│   ├── camera_publisher_node.py
│   └── aruco_localizer_node.py (old)
├── config/
│   ├── aruco_boards/           # Board configuration YAMLs
│   ├── camera_intrinsics/      # Camera calibration YAMLs
│   ├── aruco_board_params.yaml
│   └── camera_params.yaml
├── launch/
│   ├── aruco_camera_pipeline.launch.py  # Combined launch
│   ├── aruco_board_detector.launch.py
│   └── camera_publisher.launch.py
├── old_design/                 # Reference implementation
├── README.md
├── README_CAMERA_PUBLISHER.md
├── README_BOARD_DETECTOR.md
└── setup.py
```

## Troubleshooting

### Camera Issues
- Check device path: `ls -l /dev/video*`
- Check permissions: Add user to video group
- Try different device indices (video0, video1, etc.)
- Check with v4l2-ctl: `v4l2-ctl --device=/dev/video0 --all`

### ArUco Detection Issues
- Verify camera_info is publishing: `ros2 topic echo /camera/camera_info`
- Check board YAML files are in correct directory
- Ensure correct ArUco dictionary matches your markers
- View debug image to see what's being detected

### Build Issues
- Ensure interfaces package is built first
- Source workspace after building: `source install/setup.bash`
- Clean build if needed: `rm -rf build install log`

## References

- [OpenCV ArUco Documentation](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
- [ROS2 Camera Calibration](https://navigation.ros.org/tutorials/docs/camera_calibration.html)
- [Camera Info Manager](http://wiki.ros.org/camera_info_manager)
