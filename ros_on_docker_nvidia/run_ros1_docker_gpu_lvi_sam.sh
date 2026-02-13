#!/bin/bash

# Build the Docker image if not already built
docker build -t ros1-dev-gpu-base -f ros_on_docker_nvidia/Dockerfile_ros1_gpu_base .
docker build -t ros1-dev-gpu-lvisam -f ros_on_docker_nvidia/Dockerfile_ros1_gpu_lvi_sam .

# Ensure X11 forwarding is allowed
xhost +local:docker

# Run the container with minimal privileges
docker run -it --rm \
    --runtime=nvidia \
    --gpus=all \
    --env="DISPLAY=${DISPLAY}" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=${XAUTHORITY}" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/dev/dri:/dev/dri:rw" \
    --volume="`pwd`:/root/src_wip" \
    --volume="/opt/mnt/data/10_slam/:/data/" \
    --device="/dev/dri" \
    --group-add="video" \
    --group-add="render" \
    --ipc="host" \
    --network="host" \
    --name ros1-dev-gpu-container \
    ros1-dev-gpu-lvisam

# Clean up
xhost -local:docker
