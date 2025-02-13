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

### Sample Dataset

Download [Semantic KITTI Dataset](https://www.semantic-kitti.org/dataset.html#download).

Downloaded dataset structure is as follows:

```shell
ros-groundgrid
├── data
│   └── kitti
│       ├── dataset
│       |   └── sequences
|       │       ├── 00
│       │       │   ├── calib.txt
│       │       │   ├── labels
│       │       │   ├── poses.txt
│       │       │   ├── times.txt
│       │       │   └── velodyne
│       │       ├── 01
...
```

#### Requirements

`ros-humble-rosbag2-storage-mcap` is required to serialize bag files into [MCAP format](https://mcap.dev/).

```shell
sudo apt install ros-humble-rosbag2-storage-mcap
```

#### Save ROS Bag

Run `kitti2bag.py` to convert dataset to bag file in mcap format:

```shell
python3 scripts/kitti2bag.py </KITTI/DATASET/PATH>
```

For example in the above case:

```shell
python3 scripts/kitti2bag.py ./data/kitti/dataset
```

In order to convert the particular sequence, please specify `-s <SEQUENCE_NUMBER>`:

```shell
python3 scripts/kitti2bag.py </KITTI/DATASET/PATH> -s 00
```

#### Play Bag file

After finishing conversion, you can run the recorded bag file as below:

```shell
ros2 bag play -s mcap </KITTIT/BAG/DIR>
```

Bag files will be saved into `data/kitti_bag/semantic_kitti_<SEQUENCE>` by default:

```shell
ros2 bag play -s mcap ./data/kitti_bag/semantic_kitti_00
```

## Visualize

For visualization, `rviz/groundgrid.rviz` is prepared for the rviz configuration by default:

```shell
rviz2 -d ./rviz/groundgrid.rviz
```
