#!/usr/bin/env python3
"""Publish a fixed phase-2 path fixture as nav_msgs/Path."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .pure_pursuit import load_fixture, poses_from_fixture


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class TestPathPublisher(Node):
    def __init__(self) -> None:
        super().__init__('test_path_publisher')
        default_fixture_dir = str(Path(get_package_share_directory('autoracer_path_tracking')) / 'fixtures' / 'stage2_paths')
        self.declare_parameter('fixture_dir', default_fixture_dir)
        self.declare_parameter('case_name', 'straight_2m')
        self.declare_parameter('publish_rate_hz', 1.0)

        self.fixture_dir = Path(self.get_parameter('fixture_dir').value)
        self.case_name = str(self.get_parameter('case_name').value)
        publish_rate_hz = max(0.1, float(self.get_parameter('publish_rate_hz').value))

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(PathMsg, '/path_tracking/path', qos)
        self.path_msg = self._load_path()
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_path)
        self.get_logger().info(f'Publishing stage-2 path fixture {self.case_name}')

    def _load_path(self) -> PathMsg:
        fixture_path = self.fixture_dir / f'{self.case_name}.json'
        fixture = load_fixture(fixture_path)
        poses = poses_from_fixture(fixture)
        frame_id = str(fixture.get('frame_id', 'odom'))

        path = PathMsg()
        path.header.frame_id = frame_id
        for pose in poses:
            stamped = PoseStamped()
            stamped.header.frame_id = frame_id
            stamped.pose.position.x = pose.x
            stamped.pose.position.y = pose.y
            stamped.pose.position.z = 0.0
            qx, qy, qz, qw = yaw_to_quaternion(pose.yaw)
            stamped.pose.orientation.x = qx
            stamped.pose.orientation.y = qy
            stamped.pose.orientation.z = qz
            stamped.pose.orientation.w = qw
            path.poses.append(stamped)
        return path

    def publish_path(self) -> None:
        now = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = now
        for pose in self.path_msg.poses:
            pose.header.stamp = now
        self.publisher.publish(self.path_msg)


def main() -> None:
    rclpy.init()
    node = TestPathPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
