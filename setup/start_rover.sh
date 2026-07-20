#!/bin/bash
sudo enablecan.sh

ROVER_DOCKER_IMAGE=cprtsoftware/rover:latest

web_server_id=$(docker ps -q -f name=web-server)
if [ -z "$web_server_id" ]; then
    echo "Starting the web server..."
    docker run --rm -d \
        --network host \
        --ipc host \
        --name web-server \
        $ROVER_DOCKER_IMAGE \
        ros2 launch rosbridge_server rosbridge_websocket_launch.xml
else
    echo "Web server is already running."
fi

echo "Starting the container launcher..."

exec docker run --rm \
        -v /var/run/docker.sock:/var/run/docker.sock \
        -p 8080:8080 \
        --name container-launcher \
        cprtsoftware/container-launcher:latest