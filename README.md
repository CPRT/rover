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

## Known limitations:
1) When using the Dev container sometimes msgs are not recieved unless you are running as root (Recommended work around is run all ros2 commands as root inside the container)

