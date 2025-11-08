# Video Streaming
This is currently under development. For now continue using the camera_streaming package.

## Architecture:
This package is broken down into ros2 components. Each component runs a GStreamer pipeline to perform some video processing task. Interpipe elements developed by RidgeRun are used to efficiently (no-copy) pass buffers between components. This requires that all components that share buffers are run inside the same process (Ros2 container).

## Components

#### Input node
Takes in video sources and controls which ones are composited and displayed on the output image.
Inputs implemented:
* [x] Video test source (videotestsrc element)
* [x] V4l2 drivers (USB cams)
* [ ] Static overlays
* [ ] Ros2 image topics

#### Detect node
Runs object detection on the incoming video feed
Detection implemented:
* [ ] Aruco Tags
* [ ] Hammer
* [ ] Ice Pick

#### WebRTC node
This is the same method of video streaming we used in 2024/25. Easy use since it is compatible with browsers. Downside is that is was specifically made for the internet at large and has overhead that isn't needed for our use case. STATUS: Implemented

#### Image capture
Captures jpeg images and returns them on request using a ros2 service API. STATUS: Un-implemented

#### SRT node
Possible replacement for WebRTC node. Coming Soon. 