
# Docker Instructions

This is a temporary file to document how to use the Docker setup.
This should be moved into the README.md eventually.


## Outline

1. Clone
2. Open docker container
3. Build within container
4. Run elevation_mapping

## Clone
> **Note:**  It is very important to clone the repo in a ros2 workspace with a src directory.

Run the following to setup the repo:
```
mkdir -p ~/ros2_ws/src/ # create your workspace if it does not exist
cd ~/ros2_ws/src/ # use your current ros2 workspace folder
git clone https://github.com/ErikCald/elevation_mapping.git
git clone -b ros2 https://github.com/SivertHavso/kindr_ros.git
cd ..
```

## Open Docker Container

> **Note:** You can connect multiple terminals to the same container by executing the `run_docker.sh` script in each terminal.

This will build and run the container. Building the container the first is expected to take a bit of time (~15 minutes)
Run the following to open the docker container:
```
cd ~/ros2_ws/src/elevation_mapping/docker
./run_docker.sh
```

## Build within Container

Executing the run_docker.sh script will build and open the container. From within the container, run the following commands to build the elevation_mapping package:
```
cd $ELEVATION_MAPPING_DIR
colcon build --symlink-install
```

## Run elevation_mapping

I made some config files and a launch file to work with the ZED Stereo Cameras. To use the ZED setup, you will need to also setup the zed_ros2_wrapper yourself.

Here is the setup I use:

1. Terminal 1 within Container: `source $ELEVATION_MAPPING_DIR/install/setup.bash && ros2 launch elevation_mapping rviz.launch.py`
2. Terminal 2 within Container: `source $ELEVATION_MAPPING_DIR/install/setup.bash && ros2 launch elevation_mapping zed2i.launch.py`
3. Terminal 3 outside Container: `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i`

# Credit

Credit mostly to the Nvidia Isaac Ros Common repo for the docker setup. 
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common

I have modified (mostly removed extra bits) and simplified it down from the Nvidia Isaac Common. I also added nice features like grabbing all the package.xml files from the local copy of the repo and running rosdep install on them, and I added persistent bash histories.

