#!/bin/bash

SCRIPT_DIR="$(dirname "$0")"
DOCKERFILE_DIR="$SCRIPT_DIR/.."

# check if docker is installed
if ! command -v docker &> /dev/null
then
    echo "Docker could not be found. Are you in the Docker environment?"
    echo "If so please exit back to the host environment."
    echo "If not, please install Docker and try again. "
    exit 1
fi

echo "Generating rosdeps inside the docker container..."
docker build -f docker/Dockerfile.app --target rosdep-exporter --output $DOCKERFILE_DIR $DOCKERFILE_DIR
echo "Rosdeps generated successfully."