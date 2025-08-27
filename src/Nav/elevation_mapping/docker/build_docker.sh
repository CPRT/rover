#!/usr/bin/env bash

# set -x

##########################################################
### Constants 
##########################################################

PLATFORM="$(uname -m)"
IMAGE_NAME="elevation_mapping_ros2"

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
SCRIPT_NAME=$(basename "$0")
EM_REPO_DIR=$( cd "$SCRIPT_DIR/.." && pwd)
SRC_DIR=$( cd "$EM_REPO_DIR/.." && pwd)
WORKSPACE_DIR=$( cd "$SRC_DIR/.." && pwd)
WORKSPACE_NAME=$(basename "$WORKSPACE_DIR")

EM_REPO_NAME=$(basename "$EM_REPO_DIR")
KINDR_REPO_NAME="kindr_ros"
DOCKERFILE_DIR="$WORKSPACE_DIR/src/$EM_REPO_NAME/docker"


# echo "PLATFORM: $PLATFORM"
# echo "IMAGE_NAME: $IMAGE_NAME"
# echo
# echo "SCRIPT_DIR: $SCRIPT_DIR"
# echo "SCRIPT_NAME: $SCRIPT_NAME"
# echo "WORKSPACE_DIR: $WORKSPACE_DIR"
# echo "WORKSPACE_NAME: $WORKSPACE_NAME"
# echo "DOCKERFILE_DIR: $DOCKERFILE_DIR"

##########################################################
### Safety Checks 
##########################################################

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

##########################################################
### Source helpful scripts
##########################################################

if [ ! -f "$DOCKERFILE_DIR/sourced_scripts/bash_functions.sh" ]; then
    echo "Cannot find bash_functions.sh file"
    exit 5
else
    # shellcheck disable=SC1091
    source "$DOCKERFILE_DIR/sourced_scripts/bash_functions.sh" || exit 4
fi

##########################################################
### Docker Build
##########################################################

# Remove dockerignore files created by this script on exit
REMOVE_EM_DOCKERIGNORE=0
REMOVE_KINDR_DOCKERIGNORE=0
function remove_dockerfiles()
{
   # Remove the .dockerignore files
   if [ "$REMOVE_EM_DOCKERIGNORE" == "1" ]; then
      rm -f "$WORKSPACE_DIR/src/$EM_REPO_NAME/.dockerignore"
   fi
   if [ "$REMOVE_KINDR_DOCKERIGNORE" == "1" ]; then
      rm -f "$WORKSPACE_DIR/src/$KINDR_REPO_NAME/.dockerignore"
   fi
}
trap remove_dockerfiles 0

# Copy .dockerignore to temporary workspace to only copy package.xml files
if [ ! -f "$WORKSPACE_DIR/src/$EM_REPO_NAME/.dockerignore" ]; then
    cp "$DOCKERFILE_DIR/container_scripts/.dockerignore" "$WORKSPACE_DIR/src/$EM_REPO_NAME/.dockerignore"
    REMOVE_EM_DOCKERIGNORE=1
fi
if [ ! -f "$WORKSPACE_DIR/src/$KINDR_REPO_NAME/.dockerignore" ]; then
    cp "$DOCKERFILE_DIR/container_scripts/.dockerignore" "$WORKSPACE_DIR/src/$KINDR_REPO_NAME/.dockerignore"
    REMOVE_KINDR_DOCKERIGNORE=1
fi

echo
if [ "$PLATFORM" == "aarch64" ]; then
    print_info "Selecting jetson jetpack image for base image"
    # initial_image="nvcr.io/nvidia/l4t-jetpack:${L4T_VERSION}" # TODO: Make L4T verison configurable
    initial_image="nvcr.io/nvidia/l4t-jetpack:r36.4.0"
else
    print_info "Selecting ubuntu:22.04 image for base image"
    initial_image="ubuntu:22.04"
fi

print_docker_build "Building Dockerfile.ros2_humble as ros2_humble from $initial_image"
docker build -f "$DOCKERFILE_DIR"/Dockerfile.ros2_humble \
    --network host \
    --build-arg "BASE_IMAGE=$initial_image" \
    -t ros2_humble:elevation_mapping . \
    || exit 21

print_docker_build "Building Dockerfile.user as user from ros2_humble"
docker build -f "$DOCKERFILE_DIR"/Dockerfile.user \
    --network host \
    --build-arg "BASE_IMAGE=ros2_humble:elevation_mapping" \
    -t user:elevation_mapping . \
    || exit 21

print_docker_build "Building Dockerfile.elevation_mapping as final image named $IMAGE_NAME from user"
docker build -f "$DOCKERFILE_DIR"/Dockerfile.elevation_mapping \
    --network host \
    --build-context "em_repo=$WORKSPACE_DIR/src/$EM_REPO_NAME" \
    --build-context "kindr_ros_repo=$WORKSPACE_DIR/src/$KINDR_REPO_NAME" \
    --build-arg BASE_IMAGE=user:elevation_mapping \
    -t $IMAGE_NAME . \
    || exit 22

print_info "Built image $IMAGE_NAME successfully"
