#!/bin/bash
sudo enablecan.sh

ROVER_DOCKER_IMAGE=cprtsoftware/rover:latest

 disco_server_id=$(docker ps -q -f name=disco-server)
 if [ -z "$disco_server_id" ]; then
     echo "Starting the discovery server..."
    docker run --rm -d \
        --network host \
        --ipc=host \
        -v /var/run/docker.sock:/var/run/docker.sock \
        --name disco-server \
        $ROVER_DOCKER_IMAGE \
        fastdds discovery -i 0 -p 11811
else
    echo "Discovery server is already running."
fi

web_server_id=$(docker ps -q -f name=web-server)
if [ -z "$web_server_id" ]; then
    echo "Starting the web server..."
    docker run --rm -d \
        --network host \
        --ipc host \
        --name web-server \
        --env ROS_DISCOVERY_SERVER=192.168.0.55:11811 \
        --env ROS_SUPER_CLIENT=TRUE \
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