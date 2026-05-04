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
  cprtsoftware/rover:arm64 \
  <COMMAND>
```