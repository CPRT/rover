# Camera Publisher Node

This node opens a USB camera using OpenCV with V4L2 backend, reads camera calibration from a YAML file, and publishes both raw images and camera_info to ROS2 topics.

## Features

- Opens camera using OpenCV with V4L2 backend (Linux)
- Reads camera calibration from standard ROS calibration YAML format
- Publishes synchronized image and camera_info messages
- Configurable resolution, frame rate, and video format
- Optional compressed image publishing
- Fully parameterized via ROS2 parameters

## Camera Calibration Format

The node expects camera calibration files in the standard ROS camera calibration format. Example (`ErikKlarityCam.yaml`):

```yaml
image_width: 1280
image_height: 720
camera_name: ErikKlarityCam
camera_matrix:
  rows: 3
  cols: 3
  data: [986.3321089, 0.0, 640.1575794, 0.0, 981.6186107, 500.6903186, 0.0, 0.0, 1.0]
distortion_model: rational_polynomial
distortion_coefficients:
  rows: 1
  cols: 8
  data: [0.2624628475, 0.1102693035, -0.001259269067, -2.317268492e-05, 0.2807165481,
    0.2516554863, 0.1878639645, 0.1083906076]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [986.3321089, 0.0, 640.1575794, 0.0, 0.0, 981.6186107, 500.6903186, 0.0, 0.0,
    0.0, 1.0, 0.0]
```

You can generate this file using ROS camera calibration tools:
```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 \
    --ros-args -r image:=/camera/image_raw
```

## Parameters

Parameters are configured in `config/camera_params.yaml`:

- `camera_device`: Camera device path (e.g., `/dev/video0`) or index (e.g., `0`)
- `camera_name`: Name of the camera
- `image_width`: Image width in pixels (default: 1280)
- `image_height`: Image height in pixels (default: 720)
- `frame_rate`: Publishing frame rate in Hz (default: 30.0)
- `image_topic`: Topic to publish raw images (default: `/camera/image_raw`)
- `camera_info_topic`: Topic to publish camera info (default: `/camera/camera_info`)
- `camera_frame_id`: Frame ID for the camera (default: `camera_optical_frame`)
- `calibration_file`: Path to camera calibration YAML file
- `use_compressed`: Whether to also publish compressed images (default: false)
- `fourcc`: FourCC code for video format (default: `MJPG`)

## Usage

### Launch Camera Publisher

```bash
# Run with default config
ros2 launch aruco_localizer camera_publisher.launch.py
```

### Override Specific Parameters

```bash
# Use different calibration file
ros2 launch aruco_localizer camera_publisher.launch.py \
    calibration_file:=/path/to/my_camera.yaml

# Use different config file entirely
ros2 launch aruco_localizer camera_publisher.launch.py \
    params_file:=/path/to/my_params.yaml
```

### Changing Camera Settings

To change camera device, resolution, frame rate, etc., edit `config/camera_params.yaml`:

```yaml
camera_publisher_node:
  ros__parameters:
    camera_device: "/dev/video2"  # Change camera
    image_width: 1920             # Change resolution
    image_height: 1080
    frame_rate: 60.0              # Change frame rate
    fourcc: "YUYV"                # Change format
```

Then simply launch:
```bash
ros2 launch aruco_localizer camera_publisher.launch.py
```

### Launch Complete Pipeline (Camera + ArUco Detection)

```bash
ros2 launch aruco_localizer aruco_camera_pipeline.launch.py
```

This will start both the camera publisher and ArUco board detector together.

## Topics

### Published Topics

- `/camera/image_raw` (sensor_msgs/Image): Raw RGB camera images
- `/camera/camera_info` (sensor_msgs/CameraInfo): Camera calibration info
- `/camera/image_raw/compressed` (sensor_msgs/CompressedImage): Compressed JPEG images (if enabled)

## Device Configuration

### Finding Your Camera Device

List available video devices:
```bash
ls -l /dev/video*
```

Check camera capabilities:
```bash
v4l2-ctl --device=/dev/video0 --all
```

### Supported Video Formats

The node supports various FourCC formats. Common options:
- `MJPG` - Motion JPEG (compressed, efficient)
- `YUYV` - YUV 4:2:2 (uncompressed, higher quality)
- `H264` - H.264 compressed (if supported by camera)

Check supported formats for your camera:
```bash
v4l2-ctl --device=/dev/video0 --list-formats-ext
```

### Setting Camera Permissions

If you get permission errors, add your user to the video group:
```bash
sudo usermod -a -G video $USER
```

Then log out and log back in.

## Troubleshooting

### Camera not opening

1. **Check device path**: Verify the camera is at the specified device path
   ```bash
   ls -l /dev/video*
   ```

2. **Check permissions**: Make sure you have read/write access
   ```bash
   ls -l /dev/video0
   # Should show: crw-rw---- 1 root video ...
   ```

3. **Check if device is in use**: Another application might be using the camera
   ```bash
   lsof /dev/video0
   ```

4. **Try different device index**: Some cameras register multiple devices (video0, video1, etc.)

### Poor image quality

1. **Try different FourCC format**: Switch between MJPG and YUYV
2. **Adjust resolution**: Lower resolution may give better frame rate
3. **Check lighting**: Ensure adequate lighting for the camera
4. **Verify camera capabilities**: Use `v4l2-ctl` to check supported formats

### Low frame rate

1. **Reduce resolution**: Lower resolution = higher possible frame rate
2. **Use MJPG format**: Motion JPEG is more efficient than raw formats
3. **Check USB bandwidth**: USB 2.0 limits bandwidth, use USB 3.0 if available
4. **Enable compressed publishing**: Set `use_compressed: true` to reduce network load

### Camera info not publishing

1. **Check calibration file path**: Verify the file exists and path is correct
2. **Check YAML format**: Ensure the calibration file is valid YAML
3. **Check logs**: Node will log warnings if calibration file is not found

### Testing the Camera

View the camera stream:
```bash
# Using rqt_image_view
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# Check camera info
ros2 topic echo /camera/camera_info
```

Check topic publication rate:
```bash
ros2 topic hz /camera/image_raw
```

## Dependencies

- ROS2 (Humble or later)
- OpenCV with V4L2 support
- cv_bridge
- Python packages: numpy, yaml

## Complete Pipeline Example

To run the complete ArUco detection pipeline with your camera:

```bash
# 1. Launch the complete pipeline
ros2 launch aruco_localizer aruco_camera_pipeline.launch.py \
    camera_device:=/dev/video0

# 2. In another terminal, view the debug output
ros2 run rqt_image_view rqt_image_view /aruco_debug

# 3. In another terminal, monitor detected boards
ros2 topic echo /aruco_boards
```

This will:
1. Open your camera and publish images + camera_info
2. Detect ArUco boards in the camera stream
3. Publish board poses
4. Show debug visualization with detected markers and axes
