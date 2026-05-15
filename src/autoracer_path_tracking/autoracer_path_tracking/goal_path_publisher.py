#!/usr/bin/env python3
"""Publish a straight odom-frame path from the current pose to a point goal."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .pure_pursuit_tracker import quaternion_to_yaw
from .test_path_publisher import yaw_to_quaternion


class GoalPathPublisher(Node):
    def __init__(self) -> None:
        super().__init__('goal_path_publisher')
        self.declare_parameter('goal_x_m', 2.0)
        self.declare_parameter('goal_y_m', 2.0)
        self.declare_parameter('goal_tolerance_m', 0.05)
        self.declare_parameter('point_spacing_m', 0.25)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('use_current_pose', True)
        self.declare_parameter('start_x_m', 0.0)
        self.declare_parameter('start_y_m', 0.0)
        self.declare_parameter('start_yaw_rad', 0.0)

        self.goal_x_m = float(self.get_parameter('goal_x_m').value)
        self.goal_y_m = float(self.get_parameter('goal_y_m').value)
        self.goal_tolerance_m = max(0.01, float(self.get_parameter('goal_tolerance_m').value))
        self.point_spacing_m = max(0.05, float(self.get_parameter('point_spacing_m').value))
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.use_current_pose = bool(self.get_parameter('use_current_pose').value)
        self.current_x_m = float(self.get_parameter('start_x_m').value)
        self.current_y_m = float(self.get_parameter('start_y_m').value)
        self.current_yaw_rad = float(self.get_parameter('start_yaw_rad').value)
        self.have_odom = not self.use_current_pose

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(PathMsg, '/path_tracking/path', qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 20)

        publish_rate_hz = max(0.1, float(self.get_parameter('publish_rate_hz').value))
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_path)
        self.get_logger().info(
            f'Publishing point-goal path to ({self.goal_x_m:.3f}, {self.goal_y_m:.3f}) in {self.frame_id}'
        )

    def on_odom(self, msg: Odometry) -> None:
        if not self.use_current_pose:
            return
        self.current_x_m = float(msg.pose.pose.position.x)
        self.current_y_m = float(msg.pose.pose.position.y)
        orientation = msg.pose.pose.orientation
        self.current_yaw_rad = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self.have_odom = True

    def publish_path(self) -> None:
        if not self.have_odom:
            return

        dx = self.goal_x_m - self.current_x_m
        dy = self.goal_y_m - self.current_y_m
        distance = math.hypot(dx, dy)
        path_yaw = self.current_yaw_rad if distance <= 1e-6 else math.atan2(dy, dx)
        segments = max(1, int(math.ceil(distance / self.point_spacing_m)))
        now = self.get_clock().now().to_msg()

        path = PathMsg()
        path.header.stamp = now
        path.header.frame_id = self.frame_id

        for index in range(segments + 1):
            ratio = index / segments
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = self.current_x_m + dx * ratio
            pose.pose.position.y = self.current_y_m + dy * ratio
            pose.pose.position.z = 0.0
            yaw = self.current_yaw_rad if index == 0 and distance > self.goal_tolerance_m else path_yaw
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)

        self.publisher.publish(path)


def main() -> None:
    rclpy.init()
    node = GoalPathPublisher()
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
