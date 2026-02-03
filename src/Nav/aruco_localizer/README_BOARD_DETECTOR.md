# ArUco Board Detector Node

This node detects pairs of ArUco markers configured as boards and estimates their precise pose using `cv2.aruco.estimatePoseBoard`. It provides more accurate pose estimation than individual markers by using all 8 corners from two markers simultaneously.

## Features

- Detects ArUco boards composed of marker pairs
- Uses `cv2.aruco.estimatePoseBoard` for robust pose estimation with 8 corners
- Publishes board poses in the camera frame
- Configurable via YAML board configuration files
- Publishes debug images with detected axes
- Supports multiple boards simultaneously

## Configuration

### Board Configuration Files

Board configurations are stored in YAML files in `config/aruco_boards/`. Each file defines a board with two or more markers.

Example board configuration (`camera-0_ArucoBoard_Tag6-Tag7.yaml`):

```yaml
board_description: Erik Foam 45 deg Angled Aruco Board
corners:
  - [[-0.09975, 0.09975, 0.0],
     [0.09975, 0.09975, 0.0],
     [0.09975, -0.09975, 0.0],
     [-0.09975, -0.09975, 0.0]]
  - [[0.18129618626239796, 0.09963384563760873, -0.025989066682684255],
     [0.3347121723613784, 0.09637445788601445, -0.15347694615675395],
     [0.33348615537658777, -0.10308884724011268, -0.14985277693509214],
     [0.18007016927760733, -0.0998294594885184, -0.022364897461022437]]
ids: [6, 7]
tag_size: 0.1995
```

**Important**: Corners are in the board frame, ordered as:
- Top-left, top-right, bottom-right, bottom-left for each marker
- Coordinates in meters

### Parameters

Parameters are configured in `config/aruco_board_params.yaml`:

- `image_topic`: Image topic to subscribe to (default defined in `aruco_board_params.yaml`)
- `camera_info_topic`: Camera calibration topic (default defined in `aruco_board_params.yaml`)
- `camera_frame`: Camera optical frame ID (default: `camera_color_optical_frame`)
- `aruco_dictionary_id`: ArUco dictionary used (default defined in `aruco_board_params.yaml`)
- `board_config_dir`: Directory containing board YAML files (set by launch file)
- `board_topic`: Topic to publish board poses (default: `/aruco_boards`)
- `debug_image_topic`: Topic for debug visualization (default: `/aruco_debug`)
- `marker_visualization_topic`: Topic for RViz2 markers (default: `/aruco_markers_viz`)
- `marker_size`: Size of visualization spheres in meters (default: 0.05)
- `min_markers_for_board`: Minimum markers needed for pose estimation (default: 2)

## Usage

### Launch the Node

```bash
# Run with default config
ros2 launch aruco_localizer aruco_board_detector.launch.py
```

### Override Specific Parameters

```bash
# Use different board directory
ros2 launch aruco_localizer aruco_board_detector.launch.py \
    board_config_dir:=/path/to/my_boards

# Use different config file entirely
ros2 launch aruco_localizer aruco_board_detector.launch.py \
    params_file:=/path/to/my_params.yaml
```

### Changing Detection Settings

To change topics, dictionary, or other settings, edit `config/aruco_board_params.yaml`:

```yaml
aruco_board_detector_node:
  ros__parameters:
    image_topic: "/my_camera/image"
    aruco_dictionary_id: "DICT_5X5_250"
    min_markers_for_board: 1
```

Then simply launch:
```bash
ros2 launch aruco_localizer aruco_board_detector.launch.py
```

## Topics

### Subscribed Topics

- `/camera/color/image_raw` (sensor_msgs/Image): RGB camera image
- `/camera/camera_info` (sensor_msgs/CameraInfo): Camera calibration

### Published Topics

- `/aruco_boards` (interfaces/ArucoBoard): Detected board poses
  - `header`: Timestamp and frame_id
  - `board_id`: Board identifier (from YAML filename)
  - `marker_ids`: IDs of detected markers in this board
  - `pose`: Pose of board frame relative to camera frame
  - `corners_3d`: 3D positions of all board corners in camera frame

- `/aruco_debug` (sensor_msgs/Image): Visualization image with detected markers and axes

- `/aruco_markers_viz` (visualization_msgs/MarkerArray): 3D visualization markers for RViz2
  - Sphere markers at the center of each detected ArUco tag
  - Text markers showing the tag IDs
  - Color-coded: First tag (Green), Second tag (Blue), Additional tags (Yellow)

## Dependencies

- ROS2 (Humble or later)
- OpenCV 4.12.0 with ArUco module
- cv_bridge
- tf_transformations
- PyYAML
- NumPy

## Creating Board Configuration Files

To create a new board configuration:

1. Measure the 3D positions of all marker corners in the board's coordinate frame
2. Create a YAML file in `config/aruco_boards/`
3. Include:
   - `board_description`: Human-readable description
   - `ids`: List of marker IDs (e.g., [6, 7])
   - `corners`: Nested list of 3D corner positions (one list per marker)
   - `tag_size`: Size of individual markers in meters

The board frame origin can be anywhere, but corners must be specified relative to it.

## Troubleshooting

### No boards detected

- Verify camera is publishing images and camera_info
- Check that markers are from the correct dictionary
- Ensure board YAML files are in the correct directory
- Check that at least `min_markers_for_board` markers are visible

### Inaccurate pose estimates

- Verify camera calibration is accurate
- Check that board corner measurements are precise
- Ensure markers are planar and not warped
- Verify lighting conditions are good

### Debug visualization

Subscribe to `/aruco_debug` to see:
- Detected marker boundaries
- Marker IDs
- Board coordinate axes (if pose estimation succeeds)

```bash
ros2 run rqt_image_view rqt_image_view /aruco_debug
```

### 3D Visualization in RViz2

View the tag centers and IDs in RViz2:

1. Launch RViz2:
   ```bash
   rviz2
   ```

2. Add a MarkerArray display:
   - Click "Add" button
   - Select "By topic" tab
   - Choose `/aruco_markers_viz` → `MarkerArray`
   - Click "OK"

3. Set the Fixed Frame to match your camera frame (e.g., `camera_optical_frame`)

You'll see:
- **Green spheres** at the center of the first tag in each board
- **Blue spheres** at the center of the second tag in each board
- **Yellow spheres** for any additional tags
- **White text labels** showing the tag IDs above each sphere

This helps verify that:
- Both tags are being detected correctly
- Their positions relative to each other match your board configuration
- The board pose estimation is accurate
