# Preparations

## Datasets

Arrange your data so that every single rosbag is placed in a dedicated directory.

HILTI 2021:
https://hilti-challenge.com/dataset-2021.html

NTU Viral:
https://ntu-aris.github.io/ntu_viral_dataset/

## Build and run the docker image

```bash
$ ./ros_on_docker_nvidia/run_ros1_docker_gpu_mad_icp.sh
```

## Troubleshooting: OS permissions for running docker

In case of OS permission issues, check if your user is a member of the `docker` group
`groups | grep docker`
If not, then run:
```
sudo groupadd docker
sudo usermod -aG docker $USER
```

## Working tips

Start 3 consoles in docker. Recommending use of `tmux` within the docker console.
Alternatively use: `docker exec -it ros1-dev-gpu-mad-icp /bin/bash`

# Running

## Running on NTU Viral dataset

IMPORTANT: NTU Viral dataset requires output cloud transform defined in yaml as: `cloud_transform: [[1, -1, -1]]`.
TIP: when in docker, call this once:
`echo "cloud_transform: [[1, -1, -1]]" >> /catkin_ws/src/mad-icp/mad_icp/configurations/datasets/ntu_viral.cfg`

DATASET_PATH below is `/data/NTU_Viral_dataset/` please replace with your corresponding path.

Start `roscore` and then run `mad_icp`.

```
mad_icp --data-path /data/NTU_Viral_dataset/eee_02/ --estimate-path /data/NTU_Viral_dataset/eee_02/output/ --dataset-config /catkin_ws/src/mad-icp/mad_icp/configurations/datasets/ntu_viral.cfg --noviz
```

*Optional args*
`--noviz` - starts without the build-in (based on `open3d`) visualization window.
`--noros` - starts without ROS publisher

*Visualization*
```
rviz rviz -d src/mad-icp/mad_icp/configurations/rviz/mad_icp_rviz_odom_and_cloud.rviz
```

*Recording a rosbag*
```
rosbag record -o /data/NTU_Viral_dataset/MAD_ICP/eee_02_results.bag /cloud/current /cloud/complete  /odometry/imu
```

*Conversion to MapsHD session*
```
source /catkin_ws/devel/setup.bash
rosrun cpp_pubsub listener /data/NTU_Viral_dataset/MAD_ICP/eee_02_results.bag /data/NTU_Viral_dataset/MAD_ICP/eee_02_results_session/ /cloud/current
```

## HILTI 2021 dataset

DATASET_PATH below is `/data/HILTI_2021/` please replace with your corresponding path.

```
rosbag info /data/HILTI_2021/construction_site_1/Construction_Site_1.bag
rosbag info /data/HILTI_2021/uzh_tracking_area_run2/uzh_tracking_area_run2.bag
```

Dataset 1 "RPG Drone Testing Arena" (6 DoF Ground truth available):
```
mad_icp --data-path /data/HILTI_2021/construction_site_1/ --estimate-path /data/HILTI_2021/construction_site_1/output/ --dataset-config hilti_2021
```

Dataset 2 "Construction Site Outdoor 1":
```
mad_icp --data-path /data/HILTI_2021/uzh_tracking_area_run2/ --estimate-path /data/HILTI_2021/uzh_tracking_area_run2/output/ --dataset-config hilti_2021
```

If `hilti_2021` is not recognized, use full path to config as shown below:
```
mad_icp --data-path /data/HILTI_2021/uzh_tracking_area_run2/ --estimate-path /data/HILTI_2021/uzh_tracking_area_run2/output/ --dataset-config /catkin_ws/src/mad-icp/mad_icp/configurations/datasets/hilti_2021.cfg --noviz
```

*Visualization*
```
rviz rviz -d src/mad-icp/mad_icp/configurations/rviz/mad_icp_rviz_odom_and_cloud.rviz
```

*Recording a rosbag*
```
rosbag record -o /data/HILTI_2021/MAD_ICP/uzh_tracking_area_run2_results.bag /cloud/current /cloud/complete  /odometry/imu
```

*Conversion to MapsHD session*
```
source /catkin_ws/devel/setup.bash
rosrun cpp_pubsub listener /data/HILTI_2021/MAD_ICP/uhz_tracking_area_run2_results_2025-09-07-15-04-37.bag /data/HILTI_2021/MAD_ICP/uhz_tracking_area_run2_session/ /cloud/current
```