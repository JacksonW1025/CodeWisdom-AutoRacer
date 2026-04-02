#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class AutoracerImuTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__('autoracer_imu_tf_broadcaster')

        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('parent_frame', 'viz_world')
        self.declare_parameter('child_frame', 'viz_base_link')
        self.declare_parameter('translation.x', 0.0)
        self.declare_parameter('translation.y', 0.0)
        self.declare_parameter('translation.z', 0.11)

        self._imu_topic = self.get_parameter('imu_topic').value
        self._parent_frame = self.get_parameter('parent_frame').value
        self._child_frame = self.get_parameter('child_frame').value
        self._translation = (
            float(self.get_parameter('translation.x').value),
            float(self.get_parameter('translation.y').value),
            float(self.get_parameter('translation.z').value),
        )

        self._tf_broadcaster = TransformBroadcaster(self)
        self._warned_zero_quaternion = False

        self.create_subscription(
            Imu,
            self._imu_topic,
            self._imu_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Broadcasting visualization TF {self._parent_frame} -> {self._child_frame} '
            f'from {self._imu_topic}'
        )

    def _imu_callback(self, msg: Imu) -> None:
        norm = math.sqrt(
            msg.orientation.x * msg.orientation.x
            + msg.orientation.y * msg.orientation.y
            + msg.orientation.z * msg.orientation.z
            + msg.orientation.w * msg.orientation.w
        )
        if norm < 1e-6:
            if not self._warned_zero_quaternion:
                self.get_logger().warn('Received near-zero IMU quaternion; skipping TF broadcast')
                self._warned_zero_quaternion = True
            return

        transform = TransformStamped()
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            transform.header.stamp = self.get_clock().now().to_msg()
        else:
            transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self._parent_frame
        transform.child_frame_id = self._child_frame
        transform.transform.translation.x = self._translation[0]
        transform.transform.translation.y = self._translation[1]
        transform.transform.translation.z = self._translation[2]
        transform.transform.rotation.x = msg.orientation.x / norm
        transform.transform.rotation.y = msg.orientation.y / norm
        transform.transform.rotation.z = msg.orientation.z / norm
        transform.transform.rotation.w = msg.orientation.w / norm

        self._tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoracerImuTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
