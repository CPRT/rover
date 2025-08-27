#!/usr/bin/env bash

# set -x

#######################################################################################
### Script Details 
#######################################################################################

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
SCRIPT_NAME=$(basename "$0")
EM_REPO_DIR=$( cd "$SCRIPT_DIR/.." && pwd)
SRC_DIR=$( cd "$EM_REPO_DIR/.." && pwd)
WORKSPACE_DIR=$( cd "$SRC_DIR/.." && pwd)
WORKSPACE_NAME=$(basename "$WORKSPACE_DIR")

USER=$(id -u -n)
PLATFORM="$(uname -m)"
CONTAINER_USER=dev

EM_REPO_NAME=$(basename "$EM_REPO_DIR")
KINDR_REPO_NAME="kindr_ros"
DOCKERFILE_DIR="$WORKSPACE_DIR/src/$EM_REPO_NAME/docker"

#######################################################################################
### Constants 
#######################################################################################

IMAGE_NAME="elevation_mapping_ros2"
CONTAINER_NAME=

print_constants()
{
    echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
    echo "Constants used by script called $SCRIPT_NAME"
    echo ""
    echo "Script Details:"
    echo "    SCRIPT_DIR: $SCRIPT_DIR"
    echo "    SCRIPT_NAME: $SCRIPT_NAME"
    echo "    EM_REPO_DIR: $EM_REPO_DIR"
    echo "    SRC_DIR: $SRC_DIR"
    echo "    WORKSPACE_DIR: $WORKSPACE_DIR"
    echo "    WORKSPACE_NAME: $WORKSPACE_NAME"
    echo "    DOCKERFILE_DIR: $DOCKERFILE_DIR"
    echo "    USER: $USER"
    echo "    PLATFORM: $PLATFORM"
    echo "    CONTAINER_USER: $CONTAINER_USER"
    echo "    EM_REPO_NAME: $EM_REPO_NAME"
    echo "    KINDR_REPO_NAME: $KINDR_REPO_NAME"
    echo ""
    echo "Docker Constants:"
    echo "    PLATFORM: $PLATFORM"
    echo "    IMAGE_NAME: $IMAGE_NAME"
    echo "    CONTAINER_NAME: $CONTAINER_NAME"
    echo ""
    echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
}

if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "Missing workspace dir at $WORKSPACE_DIR"
    exit 1
fi

if [ ! -d "$WORKSPACE_DIR/src/$EM_REPO_NAME" ]; then
    echo "Missing elevation_mapping_ros2 repo at $WORKSPACE_DIR/src/$EM_REPO_NAME"
    exit 2
fi

if [ ! -d "$WORKSPACE_DIR/src/$KINDR_REPO_NAME" ]; then
    echo "Missing kindr ros repo at $WORKSPACE_DIR/src/$KINDR_REPO_NAME"
    exit 3
fi

if [ ! -d "$DOCKERFILE_DIR" ]; then
    echo "Missing docker dir at $DOCKERFILE_DIR"
    exit 4
fi

#######################################################################################
### Source helpful scripts
#######################################################################################

if [ ! -f "$SCRIPT_DIR/sourced_scripts/bash_functions.sh" ]; then
    echo "Cannot find bash_functions.sh file"
    exit 5
else
    # shellcheck disable=SC1091
    source "$SCRIPT_DIR/sourced_scripts/bash_functions.sh"
fi

#######################################################################################
### Argument Parser
#######################################################################################

usage()
{
    echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
    echo "Builds Dockerfile for Elevation Mapping ROS2"
    echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
    echo
    echo "options:"
    echo "    -c or --container <container_name>  Specify the container name to use."
    echo "    -b or --skip-image-build            Skip building the container."
    echo "    -p or --print-constants                   Print the script's constants and exit."
    echo "    -h or --help                        Print this help."
    echo
    echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
    echo
}

SKIP_IMAGE_BUILD=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        -c|--container)
            CONTAINER_NAME="$2"
            shift 2
            ;;
        -b|--skip-image-build)
            SKIP_IMAGE_BUILD=1
            shift
            ;;
        -p|--print-constants)
            print_constants
            exit 0
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Invalid option: $1"
            echo
            usage
            exit 6
            ;;
    esac
done

#######################################################################################
### Setup
#######################################################################################

if [ -z $CONTAINER_NAME ]; then
    CONTAINER_NAME="$IMAGE_NAME-container"
    print_warning "Container name not provided. Using default container name $CONTAINER_NAME"
fi

# Ensure persistent directory is available
mkdir -p "$SCRIPT_DIR/persistent_container_files/bash_histories"

# Ensure a bash history is available for this container
bash_history_file="$SCRIPT_DIR/persistent_container_files/bash_histories/$CONTAINER_NAME-bash_history"
if [ ! -f "$bash_history_file" ]; then
    echo 'echo "Initial Bash History for elevation_mapping_ros2 container"' > "$bash_history_file"
fi

#######################################################################################
### Safety Checks 
#######################################################################################

# Prevent running as root.
if [[ $(id -u) -eq 0 ]]; then
    print_error "This script cannot be executed with root privileges."
    print_error "Please re-run without sudo and follow instructions to configure docker for non-root user if needed."
    exit 1
fi

# Check if user can run docker without root.
RE="\<docker\>"
if [[ ! $(groups $USER) =~ $RE ]]; then
    print_error "User |$USER| is not a member of the 'docker' group and cannot run docker commands without sudo."
    print_error "Run 'sudo usermod -aG docker \$USER && newgrp docker' to add user to 'docker' group, then re-run this script."
    print_error "See: https://docs.docker.com/engine/install/linux-postinstall/"
    exit 1
fi

# Check if able to run docker commands.
if [[ -z "$(docker ps)" ]] ;  then
    print_error "Unable to run docker commands. If you have recently added |$USER| to 'docker' group, you may need to log out and log back in for it to take effect."
    print_error "Otherwise, please check your Docker installation."
    exit 1
fi

# # Check if git-lfs is installed.
# git lfs &>/dev/null
# if [[ $? -ne 0 ]] ; then
#     print_error "git-lfs is not insalled. Please make sure git-lfs is installed before you clone the repo."
#     exit 1
# fi

# Check if all LFS files are in place in the repository where this script is running from.
# cd $ROOT
# git rev-parse &>/dev/null
# if [[ $? -eq 0 ]]; then
#     LFS_FILES_STATUS=$(cd $WORKSPACE_DIR && git lfs ls-files | cut -d ' ' -f2)
#     for (( i=0; i<${#LFS_FILES_STATUS}; i++ )); do
#         f="${LFS_FILES_STATUS:$i:1}"
#         if [[ "$f" == "-" ]]; then
#             print_error "LFS files are missing. Please re-clone repos after installing git-lfs."
#             exit 1
#         fi
#     done
# fi

#######################################################################################
### Docker Setup
#######################################################################################

# Remove any exited containers.
if [ "$(docker ps -a --quiet --filter status=exited --filter name=$CONTAINER_NAME)" ]; then
    docker rm $CONTAINER_NAME > /dev/null
fi

# Re-use existing container.
if [ "$(docker ps -a --quiet --filter status=running --filter name=$CONTAINER_NAME)" ]; then
    print_info "Attaching to running container: $CONTAINER_NAME"
    # shellcheck disable=SC2068
    docker exec -i -t -u "$CONTAINER_USER" --workdir "/workspaces/$WORKSPACE_NAME" $CONTAINER_NAME /bin/bash $@
    exit 0
fi

# Summarize launch
print_info "Launching elevation mapping ros2 container with image name ${IMAGE_NAME} in ${WORKSPACE_NAME}"

#######################################################################################
### Docker Build
#######################################################################################

# Build imag to launch
if [ "$SKIP_IMAGE_BUILD" = "0" ]; then
    print_docker_run "Running build_docker.sh to build the docker image"
    # shellcheck disable=SC2086
    $DOCKERFILE_DIR/build_docker.sh
    if [ "$?" != "0" ]; then
        print_error "build_docker.sh script failed"
        exit 99
    fi

    # Check result
    if [ $? -ne 0 ]; then
        if [[ -z $(docker image ls --quiet $IMAGE_NAME) ]]; then
            print_error "Building image failed and no cached image found for $IMAGE_NAME, aborting."
            exit 1
        else
            print_error "Failed to build image. A cached image is found. Use --skip-image-build to skip the build and use this cached image."
            exit 1
        fi
    fi
fi

#######################################################################################
### Docker Run
#######################################################################################

# Check image is available
if [[ -z $(docker image ls --quiet $IMAGE_NAME) ]]; then
    print_error "No built image found for $IMAGE_NAME, aborting."
    exit 1
fi

# Map host's display socket to docker
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/$CONTAINER_USER/.Xauthority:rw")
DOCKER_ARGS+=("-e DISPLAY")
DOCKER_ARGS+=("-e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml")
DOCKER_ARGS+=("-e ROS_DOMAIN_ID")
DOCKER_ARGS+=("-e USER=$CONTAINER_USER")
DOCKER_ARGS+=("-e ELEVATION_MAPPING_DIR=/workspaces/$WORKSPACE_NAME")
DOCKER_ARGS+=("-v $SCRIPT_DIR/persistent_container_files:/home/$CONTAINER_USER/persistent_container_files")
DOCKER_ARGS+=("-e BASH_HISTORY_FILE=/home/$CONTAINER_USER/persistent_container_files/bash_histories/$CONTAINER_NAME-bash_history")
DOCKER_ARGS+=("-e PERSISTENT_DIR=/home/$CONTAINER_USER/persistent_container_files")

if [[ $PLATFORM == "aarch64" ]] || [[ $HAS_CUDA_GPU == "1" ]]; then
    DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
    DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
    DOCKER_ARGS+=("--runtime nvidia")
fi

if [[ $PLATFORM == "aarch64" ]]; then
    DOCKER_ARGS+=("-v /usr/bin/tegrastats:/usr/bin/tegrastats")
    DOCKER_ARGS+=("-v /tmp/:/tmp/")
    DOCKER_ARGS+=("-v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra")
    DOCKER_ARGS+=("-v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api")
    DOCKER_ARGS+=("--pid=host")
    DOCKER_ARGS+=("-v /usr/share/vpi3:/usr/share/vpi3")
    DOCKER_ARGS+=("-v /dev/input:/dev/input")

    # If jtop is present, give the container access
    if [[ $(getent group jtop) ]]; then
        DOCKER_ARGS+=("-v /run/jtop.sock:/run/jtop.sock:ro")
        JETSON_STATS_GID="$(getent group jtop | cut -d: -f3)"
        DOCKER_ARGS+=("--group-add $JETSON_STATS_GID")
    fi
fi

# Run container from image
print_docker_run "Running $CONTAINER_NAME"
if [[ $VERBOSE -eq 1 ]]; then
    set -x
fi

# shellcheck disable=SC2068
docker run -it --rm \
    --privileged \
    --network host \
    ${DOCKER_ARGS[@]} \
    -v "$WORKSPACE_DIR:/workspaces/$WORKSPACE_NAME" \
    -v /etc/localtime:/etc/localtime:ro \
    --name "$CONTAINER_NAME" \
    --user="$CONTAINER_USER" \
    --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh \
    --workdir "/workspaces/$WORKSPACE_NAME" \
    $IMAGE_NAME \
    /bin/bash
