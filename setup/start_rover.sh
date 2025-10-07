#!/bin/bash
sudo enablecan.sh

echo "Starting the container launcher..."

docker run --rm \
    -v /var/run/docker.sock:/var/run/docker.sock \
    -p 8080:8080 \
    cprtsoftware/container-launcher:latest