# Setup

## firstTimeInstall.sh
Runs all the setup scripts. Only for use on the Jetson after flashing.

## install_ros_humble.sh (Required)
Installs ros2 humble and additional dependancies needed for all members. Works on x86 and arm64 (ubuntu 22.04)

## install_gstreamer.sh (Optional)
Installs gstreamer dependancies needed for members working on video streaming. Works on x86 and arm64 (ubuntu 22.04)

## install_nav_deps.sh (Optional)
Installs the zed sdk and kindr dependancies needed for members working on navigation. Works on x86 and arm64 (ubuntu 22.04)
Requires Nvidia GPU

## setup_service.sh (Rover Only)
Sets up a linux service that starts the core rover elements. The node manager can then be used to activate other packages.
Recommended that this runs on the rover only.

## start_rover.service
The linux service file that starts the core rover elements

## start_rover.sh
Script that starts core rover elements