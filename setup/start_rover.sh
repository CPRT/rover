#!/bin/bash
sudo enablecan.sh

 disco_server_id=$(docker ps -q -f name=disco-server)
 if [ -z "$disco_server_id" ]; then
     echo "Starting the discovery server..."
    docker run --rm -d \
        --network host \
        --ipc=host \
        -v /var/run/docker.sock:/var/run/docker.sock \
        --name disco-server \
        cprtsoftware/rover:arm64 \
        fastdds discovery -i 0 --port 11811
else
    echo "Discovery server is already running."
fi


echo "Starting the container launcher..."

docker run --rm \
    -v /var/run/docker.sock:/var/run/docker.sock \
    -p 8080:8080 \
    --name container-launcher \
    cprtsoftware/container-launcher:latest
