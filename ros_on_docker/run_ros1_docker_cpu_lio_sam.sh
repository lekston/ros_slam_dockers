#!/bin/bash

# Build the Docker image if not already built
docker build -t ros1-noetic-cpu-upgraded -f ros_on_docker/Dockerfile_ros1_noetic_cpu_base .
docker build -t ros1-noetic-cpu-lio-sam-final -f ros_on_docker/Dockerfile_ros1_lio_sam .

# Ensure X11 forwarding is allowed
xhost +local:docker

# Run the container with X11 forwarding and NVIDIA GPU access
docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="`pwd`:/root/src_wip" \
    --volume="/opt/mnt/data/10_slam/:/data/" \
    --network=host \
    --name ros1-cpu-lio-sam \
    ros1-noetic-cpu-lio-sam-final

# Clean up
xhost -local:docker