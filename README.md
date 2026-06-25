# CPRT Rover
Carleton's Planetary Robotics Team (CPRT) code for rover side operations.

## Dependencies

- **Docker**: Required to build and run the development container.
- **Git**: Needed to clone the repository and its submodules.4
- **WSL**: Needed to run docker desktop (Windows only)*

## Getting Started
Ensure you have an SSH key associated with you github account

To clone this repository along with all its submodules, run:

```bash
git clone --recurse-submodules git@github.com:CPRT/rover.git
```

#### Running Dev container
1) Download the VS Code Remote Extension
2) Click the remote explorer button in bottom left corner
3) Select re-open in container
4) Select config file consistent with your setup (amd64 for linux x86, jetson for arm64, or windows for windows x86)
5) Source ros.sh to source the ros2 virtual environment
```bash
. ros.sh
```

## When using alongside webUI launching
  - Make sure to set `ROS_DISCOVERY_SERVER=<ip of rover>`
  - To make `ros2 topic list` work, set `ROS_SUPER_CLIENT=TRUE`
  - Restart the Docker daemon

```bash
export ROS_DISCOVERY_SERVER=192.168.0.55:11811
export ROS_SUPER_CLIENT=TRUE
ros2 daemon stop
ros2 daemon start
```

### Launching a docker container like the webUI
```bash
docker run -it \
  --privileged \
  --network host \
  --ipc host \
  --rm \
  -e ROS_DISCOVERY_SERVER=192.168.0.55:11811 \
  --runtime=nvidia \
  --volume /dev/serial/by-id:/dev/serial/by-id \
  --volume /dev/v4l/by-id:/dev/v4l/by-id \
  --volume /usr/local/zed:/usr/local/zed \
  cprtsoftware/rover:latest \
  <COMMAND>
```

# Building the Docker

### Setup
To build the docker images you need to make sure to use the default buildx builder
```bash
docker buildx use default
```

### Additional tools in the docker image
#### Linter
This stage checks for pylint, black and clang-format checks to keep consistency across the codebase. Note it is possible that it fails in the MR ci/cd pipelines if new dependancies are added since the base image will not be rebuilt yet. If it still fails during the push pipeline then something is actually wrong. TODO: Fix this so it will catch it in the PR pipeline. 
```bash
./docker_build --test
```

#### rosdep exporter
This is used to generate the rosdep-keys.txt file
```bash
  ./setup/generate_rosdeps.sh
```


# Upgrade to Ubuntu 24.04 / Jetpack 7

## Migration Checklist
* [x] Move base images to Ubuntu 24.04
* [x] Update all ROS 2 build stages
* [x] Migrate GStreamer builds to binary packages
* [x] Update docker build scripts
* [x] Update DeepStream
* [x] Upgrade the ZED wrapper
* [x] Get full dev image building
* [x] Update dev container configs
* [x] Get code building inside dev container (Update c++ API calls)
* [x] Get the full images building locally
* [x] Get the -lite images building locally
* [x] Create a local docker build option (Build without internet or volume mounting)
* [x] Get pylint passing
* [x] Switch build machine to default docker builder
* [x] Get PR pipeline working
* [x] Replace all references to ROS 2 Humble
* [x] Update Jetson setup scripts
* [x] Clean up git history
* [ ] Get build pipeline working
* [ ] Reflash Jetson AGX with JP7 
* [ ] Reflash Jetson NANO with JP7 
* [x] Update Readme and other documentation

## High risk verification
* [ ] Zed Camera node
* [ ] Video Streaming
* [ ] Aruco detection
* [ ] Moveit Servo
* [ ] Nav2 stack
* [ ] WebUI
* [ ] Windows dev container