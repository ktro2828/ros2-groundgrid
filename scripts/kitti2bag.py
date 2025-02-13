# cspell: ignore velodyne, XYZI, XYZIL

# -*- coding: utf-8 -*-
import argparse
import asyncio
import glob
import os
import os.path as osp
from typing import Any, Coroutine

import numpy as np
import rosbag2_py
import tf_transformations as tf
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from numpy.typing import NDArray
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from tqdm import tqdm


class SemanticKitti2bag2:
    """Class to convert SemanticKITTI dataset to ROS BAG2 format"""

    TOPIC_NAMESPACE = "/kitti"
    WORLD_FRAME_ID = "map"
    EGO_FRAME_ID = "base_link"

    def __init__(self, data_root: str, sequence: str, save_dir: str) -> None:
        if not osp.exists(data_root):
            raise FileNotFoundError(f"Dataset path not found: {data_root}")

        if not osp.exists(save_dir):
            os.makedirs(save_dir)

        self.sequence = sequence

        sequence_path = osp.join(data_root, "sequences", str(sequence))
        self.timestamps_path = osp.join(sequence_path, "times.txt")
        self.label_path = osp.join(sequence_path, "labels")
        self.velodyne_path = osp.join(sequence_path, "velodyne")
        self.poses_path = osp.join(sequence_path, "poses.txt")

        # writer
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(
                uri=f"{save_dir}/semantic_kitti_{sequence}",
                storage_id="mcap",
            ),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )

        # PointCloud XYZI
        self.pointcloud_xyzi_topic = osp.join(self.TOPIC_NAMESPACE, "pointcloud/xyzi")
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=self.pointcloud_xyzi_topic,
                type="sensor_msgs/msg/PointCloud2",
                serialization_format="cdr",
            )
        )
        # PointCloud XYZIL
        self.pointcloud_xyzil_topic = osp.join(self.TOPIC_NAMESPACE, "pointcloud/xyzil")
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=self.pointcloud_xyzil_topic,
                type="sensor_msgs/msg/PointCloud2",
                serialization_format="cdr",
            )
        )
        # Odometry
        self.odometry_topic = osp.join(self.TOPIC_NAMESPACE, "localization/odometry")
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=self.odometry_topic,
                type="nav_msgs/msg/Odometry",
                serialization_format="cdr",
            )
        )

        # TF
        self.tf_topic = "tf"
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=self.tf_topic,
                type="tf2_msgs/msg/TFMessage",
                serialization_format="cdr",
            )
        )

    @staticmethod
    def _load_timestamps(timestamps_path: str) -> list[float]:
        """Load timestamps from file"""
        if not osp.exists(timestamps_path):
            raise FileNotFoundError(f"Timestamps path not found: {timestamps_path}")

        timestamps_list = []
        with open(timestamps_path, encoding="utf-8") as timestamps_file:
            for line in timestamps_file:
                number = float(line)
                timestamps_list.append(number)
        return timestamps_list

    @staticmethod
    def _load_poses(poses_path: str) -> list[NDArray]:
        """Load poses."""
        if not osp.exists(poses_path):
            raise FileNotFoundError(f"Poses path not found: {poses_path}")

        calibration = np.array(
            [
                [4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02],
                [-7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02],
                [9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01],
                [0, 0, 0, 1],
            ]
        )
        calibration_inv = np.linalg.inv(calibration)

        poses_list = []
        with open(poses_path, encoding="utf-8") as f:
            lines = f.readlines()
            for line in lines:
                pose = np.fromstring(line, dtype=np.float64, sep=" ")
                pose = pose.reshape(3, 4)
                pose = np.vstack((pose, [0, 0, 0, 1]))
                poses_list.append(np.matmul(calibration_inv, np.matmul(pose, calibration)))

        return poses_list

    @staticmethod
    def _get_velodyne_path_list(path: str) -> list[str]:
        """Get velodyne path list"""
        if not osp.exists(path):
            raise FileNotFoundError(f"Velodyne path not found: {path}")
        return sorted(glob.glob(path + "/*.bin"))

    @staticmethod
    def _get_labels_path_list(path: str) -> list[str]:
        """Get labels path list"""
        if not osp.exists(path):
            raise FileNotFoundError(f"Labels path not found: {path}")
        return sorted(glob.glob(path + "/*.label"))

    async def _save_pointcloud_xyzi(
        self,
        velodyne_path_list: list[str],
        timestamps_list: list[float],
    ) -> Coroutine[Any, Any, None]:
        """Save velodyne data in XYZI format to bagfile."""
        for velodyne_path, timestamp in tqdm(
            zip(velodyne_path_list, timestamps_list, strict=True),
            desc=f"Saving PointCloudXYZI in sequence: {self.sequence}",
            total=len(velodyne_path_list),
            leave=True,
        ):
            velodyne_data = np.fromfile(velodyne_path, dtype=np.float32).reshape(-1, 4)

            header = Header()
            header.frame_id = self.EGO_FRAME_ID
            header.stamp.sec = int(timestamp)
            header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)

            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            point_cloud2_msg = point_cloud2.create_cloud(header, fields, velodyne_data)

            timestamp_nanosec = int(timestamp * 1e9)
            self.writer.write(
                self.pointcloud_xyzi_topic,
                serialize_message(point_cloud2_msg),
                timestamp_nanosec,
            )

            await asyncio.sleep(0)

    async def _save_pointcloud_xyzil(
        self,
        velodyne_path_list: list[str],
        timestamps_list: list[float],
        labels_path_list: list[str],
    ) -> Coroutine[Any, Any, None]:
        """Save velodyne data and label in XYZIL format to bagfile."""
        for velodyne_path, timestamp, label_path in tqdm(
            zip(velodyne_path_list, timestamps_list, labels_path_list, strict=True),
            desc=f"Saving PointCloudXYZIL in sequence: {self.sequence}",
            total=len(velodyne_path_list),
            leave=True,
        ):
            velodyne_data = np.fromfile(velodyne_path, dtype=np.float32).reshape(-1, 4)
            label_data = np.fromfile(label_path, dtype=np.uint32).reshape(-1, 1)
            label_data = (label_data & 0xFFFF).astype(np.uint16)
            points = np.empty(
                len(velodyne_data),
                dtype=np.dtype(
                    [
                        ("x", np.float32),
                        ("y", np.float32),
                        ("z", np.float32),
                        ("intensity", np.float32),
                        ("label", np.uint16),
                    ]
                ),
            )
            points["x"] = velodyne_data[:, 0]
            points["y"] = velodyne_data[:, 1]
            points["z"] = velodyne_data[:, 2]
            points["intensity"] = velodyne_data[:, 3]
            points["label"] = label_data[:, 0]

            header = Header()
            header.frame_id = self.EGO_FRAME_ID
            header.stamp.sec = int(timestamp)
            header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)

            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name="label", offset=16, datatype=PointField.UINT16, count=1),
            ]
            point_cloud2_msg = point_cloud2.create_cloud(header, fields, points)

            timestamp_nanosec = int(timestamp * 1e9)
            self.writer.write(
                self.pointcloud_xyzil_topic,
                serialize_message(point_cloud2_msg),
                timestamp_nanosec,
            )

            await asyncio.sleep(0)

    async def _save_odometry(
        self, poses_list: list[NDArray], timestamps_list: list[float]
    ) -> Coroutine[Any, Any, None]:
        for pose, timestamp in tqdm(
            zip(poses_list, timestamps_list, strict=True),
            desc=f"Saving Odometry&TF in sequence: {self.sequence}",
            total=len(poses_list),
            leave=True,
        ):
            timestamp_nanosec = int(timestamp * 1e9)

            # odometry
            odometry_msg = Odometry()
            odometry_msg.header.frame_id = self.WORLD_FRAME_ID
            odometry_msg.header.stamp.sec = int(timestamp)
            odometry_msg.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)

            odometry_msg.pose.pose.position.x = float(pose[0][3])
            odometry_msg.pose.pose.position.y = float(pose[1][3])
            odometry_msg.pose.pose.position.z = float(pose[2][3])

            q = tf.quaternion_from_matrix(pose)
            odometry_msg.pose.pose.orientation.x = float(q[0])
            odometry_msg.pose.pose.orientation.y = float(q[1])
            odometry_msg.pose.pose.orientation.z = float(q[2])
            odometry_msg.pose.pose.orientation.w = float(q[3])

            self.writer.write(
                self.odometry_topic,
                serialize_message(odometry_msg),
                timestamp_nanosec,
            )

            # tf
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = self.WORLD_FRAME_ID
            tf_msg.header.stamp.sec = int(timestamp)
            tf_msg.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
            tf_msg.child_frame_id = self.EGO_FRAME_ID

            tf_msg.transform.translation.x = float(pose[0][3])
            tf_msg.transform.translation.y = float(pose[1][3])
            tf_msg.transform.translation.z = float(pose[2][3])

            tf_msg.transform.rotation.x = float(q[0])
            tf_msg.transform.rotation.y = float(q[1])
            tf_msg.transform.rotation.z = float(q[2])
            tf_msg.transform.rotation.w = float(q[3])

            transforms_msg = TFMessage()
            transforms_msg.transforms.append(tf_msg)

            self.writer.write(self.tf_topic, serialize_message(transforms_msg), timestamp_nanosec)

            await asyncio.sleep(0)

    async def schedule_tasks(self) -> Coroutine[Any, Any, None]:
        """Schedule tasks for creating bag file."""
        print(f">> Start converting sequence: {self.sequence}")

        timestamps_list = self._load_timestamps(self.timestamps_path)
        velodyne_path_list = self._get_velodyne_path_list(self.velodyne_path)
        labels_list = self._get_labels_path_list(self.label_path)
        poses_list = self._load_poses(self.poses_path)

        # Run each task asynchronously
        await asyncio.gather(
            self._save_pointcloud_xyzi(velodyne_path_list, timestamps_list),
            self._save_pointcloud_xyzil(velodyne_path_list, timestamps_list, labels_list),
            self._save_odometry(poses_list, timestamps_list),
        )

        print(f">> Finish converting sequence: {self.sequence}")


async def main() -> None:
    parser = argparse.ArgumentParser(description="Convert SemanticKITTI dataset to ROS BAG2 format")
    parser.add_argument("data_root", type=str, help="Path to the dataset")
    parser.add_argument(
        "-s",
        "--sequence",
        type=str,
        default="all",
        help="Sequence number to convert",
    )
    parser.add_argument(
        "-o",
        "--output",
        default="data/kitti_bag",
        type=str,
        help="Path to save converted bag files.",
    )

    args = parser.parse_args()

    if args.sequence == "all":
        # Assuming there are 22 sequences from 00 to 21
        sequences = [str(i).zfill(2) for i in range(22)]
    else:
        sequences = [args.sequence]

    tasks = []
    for sequence in sequences:
        kitti2bag = SemanticKitti2bag2(
            data_root=args.data_root, sequence=sequence, save_dir=args.output
        )
        tasks.append(kitti2bag.schedule_tasks())

    await asyncio.gather(*tasks)


if __name__ == "__main__":
    asyncio.run(main())
