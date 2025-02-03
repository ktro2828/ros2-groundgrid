# ros2-groundgrid

ROS 2 support of GroundGrid: [dcmlr/groundgrid](https://github.com/dcmlr/groundgrid).

Paper: [GroundGrid: LiDAR Point Cloud Segmentation and Terrain Estimation; IEEE'2024](https://arxiv.org/abs/2405.15664)

## Requirements

- ROS 2 Humble

## Build & Run

### Build

```shell
rosdep update
rosdep install -y --from-paths . --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Run

```shell
ros2 launch groundgrid groundgrid.launch.xml
```
