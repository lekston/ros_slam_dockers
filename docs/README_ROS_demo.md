# ROS2 Docker Development Environment

This guide provides instructions for setting up a ROS2 Humble development environment with Docker, including additional developer tools (vim and tmux) and examples for running ROS2 applications.

## Dockerfile Setup

Create the following files in your project directory:

### 1. Dockerfile

```dockerfile
FROM osrf/ros:humble-desktop

#Install vim and tmux
RUN apt-get update && apt-get install -y \
    vim \
    tmux \
    ros-humble-turtlebot3-simulations \
    && rm -rf /var/lib/apt/lists/

# Create a workspace directory
RUN mkdir -p /ros2_ws/src

# Set the working directory
WORKDIR /ros2_ws

# Set the TURTLEBOT3_MODEL environment variable
ENV TURTLEBOT3_MODEL=waffle

# Setup entrypoint
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
```
### 2. ROS Entrypoint Script

```bash
#!/bin/bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
exec "$@"
```

### 3. Docker Run Script

Run the container with X11 forwarding

```bash
docker build -t ros2-dev .

docker run -it --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--network=host \
--name ros2-dev-container \
ros2-dev
```

Make the script executable:

```bash
chmod +x run_ros2_docker.sh
```


## Building and Running the Container

1. Place all three files (Dockerfile, ros_entrypoint.sh, and run_ros2_docker.sh) in the same directory
2. Run the script to build and launch the container:
   ```bash
   ./run_ros2_docker.sh
   ```

## ROS2 Examples

Once inside the container, you can try these ROS2 examples:

### Basic Turtlesim Example

1. Launch the turtlesim node:
   ```bash
   ros2 run turtlesim turtlesim_node
   ```

2. In another terminal (using tmux), run the teleop node:
   ```bash
   ros2 run turtlesim turtle_teleop_key
   ```

3. List all available topics:
   ```bash
   ros2 topic list
   ```

4. Echo the turtle's pose:
   ```bash
   ros2 topic echo /turtle1/pose
   ```

5. View information about a topic:
   ```bash
   ros2 topic info /turtle1/cmd_vel
   ```

### TF utils

```bash
rosrun tf view_frames
rosrun tf tf_monitor
```

### Turtle with Lidar Simulation

For a more advanced example with a lidar, use the turtlebot3 simulator:

1. Install turtlebot3 packages (already provided in the Dockerfile)

2. Set the turtlebot model:
   ```bash
   export TURTLEBOT3_MODEL=waffle
   ```

3. Launch the simulation:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

4. In another terminal, run the teleop node:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   # ros2 run turtlebot3_teleop teleop_keyboard
   ```

5. View the lidar scan data:
   ```bash
   ros2 topic echo /scan
   ```

6. Visualize using RViz:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch.py
   ```

## Useful tmux Commands

Since the development environment includes tmux, here are some basic commands:

- `tmux` - Start a new session
- `Ctrl+b c` - Create a new window
- `Ctrl+b "` - Split pane horizontally
- `Ctrl+b %` - Split pane vertically
- `Ctrl+b arrow` - Navigate between panes
- `Ctrl+b d` - Detach from session
- `tmux attach` - Reattach to session

These configurations provide a solid development environment for ROS2 with convenient terminal tools.


## Container experiments with gzserver and gzclient

```bash
gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so
```

```bash
ros2 run gazebo_ros spawn_entity.py -entity waffle -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf
```

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```


## Running Docker with NVIDIA GPU Access

To run your ROS2 Docker container with NVIDIA GPU acceleration, you'll need to set up NVIDIA Container Toolkit and modify your Docker run command. Here's how to do it:

1. Install NVIDIA Container Toolkit on the host

```bash
# Add the NVIDIA repository
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

# Update and install
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Restart Docker
sudo systemctl restart docker
```

2. Option A: Modify the ROS2 docker to use the NVIDIA Container Toolkit

```bash
FROM nvidia/cuda:11.8.0-base-ubuntu22.04

# Install NVIDIA GL libraries
RUN apt-get update && apt-get install -y \
    vim \
    tmux \
    ros-humble-turtlebot3-simulations \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-plugins \
    libgl1-mesa-glx \
    libglvnd0 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    && rm -rf /var/lib/apt/lists/

# Create a workspace directory
RUN mkdir -p /ros2_ws/src

# Set the working directory
WORKDIR /ros2_ws

# Set environment variables
ENV TURTLEBOT3_MODEL=waffle
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ENV GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/opt/ros/humble/share/turtlebot3_gazebo
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Setup entrypoint
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
```

3. Update the docker run script

Modify your run script to use the NVIDIA runtime:

```bash
#!/bin/bash

docker build -t ros2-dev-nvidia .

docker run -it --rm \
  --gpus all \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --network=host \
  --name ros2-dev-container \
  ros2-dev-nvidia
```
The key addition here is the --gpus all flag, which enables GPU access inside the container.

4. Option B: Alternatively, a new ROS2 docker can use the NVIDIA CUDA base image

```bash
FROM nvidia/cuda:11.8.0-base-ubuntu22.04

# Install ROS2 Humble
RUN apt-get update && apt-get install -y locales curl gnupg2 lsb-release
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-turtlebot3-simulations \
    ros-humble-gazebo-ros-pkgs \
    vim \
    tmux \
    && rm -rf /var/lib/apt/lists/

# Create a workspace directory
RUN mkdir -p /ros2_ws/src

# Set the working directory
WORKDIR /ros2_ws

# Set environment variables
ENV TURTLEBOT3_MODEL=waffle
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ENV GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/opt/ros/humble/share/turtlebot3_gazebo

# Setup entrypoint
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
```

5. Verify GPU access

```bash
nvidia-smi
```

6. Optimize Gazebo Performance

For better Gazebo performance with NVIDIA GPUs:
1. Set additional environment variables in your Dockerfile:

```bash
   ENV LIBGL_ALWAYS_INDIRECT=0
   ENV MESA_GL_VERSION_OVERRIDE=3.3
```

2. You can also adjust Gazebo settings for better performance:

```bash
   echo "export GAZEBO_GPU_RAY_TRACE=0" >> ~/.bashrc
   echo "export OGRE_RTT_MODE=Copy" >> ~/.bashrc
```

These modifications should significantly improve Gazebo's performance by offloading rendering and physics calculations to your NVIDIA GPU.

# Running gzclient on the Host
We can also run gzclient on the host while running gzserver in the container, which can significantly improve performance. This approach is called "distributed Gazebo" and is particularly useful when the container is running on a remote machine or when you want to offload rendering to the host.

## 1. Configure the Container to Run Only gzserver
Modify your Docker run script to expose the necessary Gazebo communication ports:

```bash
#!/bin/bash

docker build -t ros2-dev-nvidia .

docker run -it --rm \
  --gpus all \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --network=host \
  --name ros2-dev-container \
  ros2-dev-nvidia
```

Using --network=host is the simplest approach as it shares the host's network stack with the container, making all ports automatically available.

## 2. Inside the Container, Run Only gzserver

When you want to run a simulation, start only the server component in the container:

```bash
# In the container
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false
```

The gui:=false parameter tells the launch file not to start gzclient.

Alternatively, you can manually start just the server with ROS integration:

```bash
# In the container
gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so /opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world
```

## 3. Run gzclient on the Host

Now, you can run gzclient on the host and connect to the server running in the container. You'll need to have Gazebo installed on your host machine:

```bash
# For Ubuntu
sudo apt-get install gazebo
```

Then, run the client:
```bash
gzclient
```

The client will automatically connect to the server running in the container.

## 4. Environment Variables for Distributed Gazebo
To ensure proper communication between the server and client, you may need to set these environment variables on both the host and in the container:

```bash
# Set these in both places
export GAZEBO_MASTER_URI=http://localhost:11345
export GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org
```


## 5. Potential Issues and Solutions

1. Version Mismatch: Ensure the Gazebo versions on the host and in the container are compatible

2. Network Configuration: If --network=host doesn't work for your setup, you can explicitly expose the Gazebo ports:
```bash
   docker run -it --rm \
     --gpus all \
     -p 11345:11345 \
     --name ros2-dev-container \
     ros2-dev-nvidia
```

3. Model Path Issues: If models don't appear correctly, you may need to install the same model packages on the host or set up a shared volume for models.

This distributed approach can dramatically improve Gazebo performance while still allowing you to keep your ROS2 environment containerized.


# Converting ROS1 bags to ROS2 bags

Adjust network settings on the host to allow handling large payloads (e.g. Livox LiDARs)
```bash
sudo sysctl -w net.ipv4.ipfrag_time=3
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728
sudo sysctl -w net.core.rmem_max=2147483647
```


```bash
sudo docker run -it --rm \
   --gpus all \
   --env="DISPLAY" \
   --env="QT_X11_NO_MITSHM=1" \
   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
   --volume="`pwd`:/ros2_ws/src_wip" \
   --volume="/opt/mnt/data/10_slam/:/ros2_ws/data/" \
   --network=host \
   --name ros1-ros2-bridge \
   ros1-ros2-dev
```

Play:
```bash
source /opt/ros/noetic/setup.bash  # ROS1 must be sourced first
source /opt/ros/foxy/setup.bash
ros2 bag play -s rosbag_v2 --read-ahead-queue-size 30000 /ros2_ws/data/TIERS/forest01_st_square_2022-02-08-23-14-55.bag
```

Recording:
```bash
ros2 bag record -a -o /ros2_ws/data/TIERS/forest01_st_square_2022-02-08-ros2
```

Test using the latest ROS2 bag player:
```bash
ros2 bag play --read-ahead-queue-size 30000 /ros2_ws/data/TIERS/forest01_st_square_2022-02-08-ros2
```

Play IMU topics only (debug using low RAM)
```bash
ros2 bag play -s rosbag_v2 /ros2_ws/data/TIERS/forest01_st_square_2022-02-08-23-14-55.bag --topics /os_cloud_node/imu /livox/imu /avia/livox/imu
```

Record IMU topics only (debug using low RAM)
```bash
ros2 bag record -o /ros2_ws/data/TIERS/forest01_st_square_2022-02-08-ros2-imu /os_cloud_node/imu /livox/imu /avia/livox/imu
```

# Running Docker Compose

```bash
docker-compose up -d
```

```bash
docker-compose down
```

```bash
docker-compose exec ros2-dev bash
```

### WIP

ros2 bag record -o /data/LIO_SAM/walking_dataset/ /imu_raw /imu_correct /points_raw /gx5/gps/fix /gx5/nav/odom /gx5/nav/status

ros2 bag play -s rosbag_v2 --read-ahead-queue-size 30000 /data/LIO_SAM/walking_dataset.bag -r 0.5