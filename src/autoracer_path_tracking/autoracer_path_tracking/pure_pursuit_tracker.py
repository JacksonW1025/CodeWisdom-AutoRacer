#!/usr/bin/env python3
"""ROS2 Pure Pursuit tracker that outputs Ackermann chassis commands."""

from __future__ import annotations

import math

import rclpy
from autoracer_interfaces.msg import AckermannChassisCommand, ChassisState, PathTrackingDiagnostics
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .pure_pursuit import ControllerConfig, Pose2D, PurePursuitController


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PurePursuitTracker(Node):
    def __init__(self) -> None:
        super().__init__('pure_pursuit_tracker')
        self.declare_parameter('case_name', '')
        self.declare_parameter('wheelbase_m', 0.60)
        self.declare_parameter('max_steering_angle_rad', 0.262)
        self.declare_parameter('lookahead_m', 0.60)
        self.declare_parameter('goal_tolerance_m', 0.05)
        self.declare_parameter('target_speed_mps', 0.20)
        self.declare_parameter('max_target_speed_mps', 0.25)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('allow_reverse', False)
        self.declare_parameter('prearm_zero_before_motion', False)
        self.declare_parameter('prearm_timeout_s', 3.0)

        self.case_name = str(self.get_parameter('case_name').value)
        self.prearm_zero_before_motion = bool(self.get_parameter('prearm_zero_before_motion').value)
        self.prearm_timeout_s = max(0.0, float(self.get_parameter('prearm_timeout_s').value))
        self.config = ControllerConfig(
            wheelbase_m=float(self.get_parameter('wheelbase_m').value),
            max_steering_angle_rad=float(self.get_parameter('max_steering_angle_rad').value),
            lookahead_m=float(self.get_parameter('lookahead_m').value),
            goal_tolerance_m=float(self.get_parameter('goal_tolerance_m').value),
            target_speed_mps=float(self.get_parameter('target_speed_mps').value),
            max_target_speed_mps=float(self.get_parameter('max_target_speed_mps').value),
            allow_reverse=bool(self.get_parameter('allow_reverse').value),
        )
        self.controller = PurePursuitController(self.config)
        self.current_pose: Pose2D | None = None
        self.last_chassis_state: ChassisState | None = None
        self.actual_speed_mps = 0.0
        self.prearm_started_ns: int | None = None
        self.prearm_complete = not self.prearm_zero_before_motion

        self.path_sub = self.create_subscription(PathMsg, '/path_tracking/path', self.on_path, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.chassis_sub = self.create_subscription(ChassisState, '/chassis_state', self.on_chassis_state, 10)
        self.command_pub = self.create_publisher(AckermannChassisCommand, '/ackermann_cmd', 10)
        self.diagnostics_pub = self.create_publisher(PathTrackingDiagnostics, '/path_tracking/diagnostics', 10)

        control_rate_hz = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self.timer = self.create_timer(1.0 / control_rate_hz, self.on_timer)
        self.get_logger().info('Pure Pursuit tracker ready')

    def on_path(self, msg: PathMsg) -> None:
        poses: list[Pose2D] = []
        for stamped in msg.poses:
            orientation = stamped.pose.orientation
            poses.append(
                Pose2D(
                    float(stamped.pose.position.x),
                    float(stamped.pose.position.y),
                    quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w),
                )
            )
        if len(poses) < 2:
            self.get_logger().warn('Ignoring path with fewer than two poses')
            return
        self.controller.set_path(poses, self.case_name)

    def on_odom(self, msg: Odometry) -> None:
        orientation = msg.pose.pose.orientation
        self.current_pose = Pose2D(
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w),
        )
        if self.actual_speed_mps == 0.0:
            self.actual_speed_mps = float(msg.twist.twist.linear.x)

    def on_chassis_state(self, msg: ChassisState) -> None:
        self.last_chassis_state = msg
        self.actual_speed_mps = float(msg.actual_speed_mps)

    def on_timer(self) -> None:
        if self.current_pose is None or not self.controller.poses:
            return

        output = self.controller.compute(self.current_pose, self.actual_speed_mps)
        stamp = self.get_clock().now().to_msg()

        if self._should_hold_for_prearm(output):
            self._publish_prearm_zero(stamp, output)
            return

        self._publish_control(stamp, output)

    def _should_hold_for_prearm(self, output) -> bool:
        if self.prearm_complete:
            return False
        if output.stop_commanded or abs(float(output.speed_mps)) <= 1e-6:
            return False
        if self._chassis_accepts_auto_control():
            self.prearm_complete = True
            self.get_logger().info('Chassis prearm complete; releasing path tracker motion commands')
            return False

        now_ns = self.get_clock().now().nanoseconds
        if self.prearm_started_ns is None:
            self.prearm_started_ns = now_ns
            self.get_logger().info('Holding zero Ackermann command until chassis auto control is ready')
        elif self.prearm_timeout_s > 0.0:
            elapsed_s = (now_ns - self.prearm_started_ns) / 1e9
            if elapsed_s >= self.prearm_timeout_s:
                self.get_logger().warn(
                    'Chassis prearm not ready; holding zero command before non-zero motion',
                    throttle_duration_sec=2.0,
                )

        return True

    def _chassis_accepts_auto_control(self) -> bool:
        if self.last_chassis_state is None:
            return False
        return (
            bool(self.last_chassis_state.auto_enabled)
            and not bool(self.last_chassis_state.command_timeout)
            and not bool(self.last_chassis_state.rc_override_active)
            and not bool(self.last_chassis_state.estop_active)
        )

    def _publish_prearm_zero(self, stamp, output) -> None:
        command = AckermannChassisCommand()
        command.header.stamp = stamp
        command.speed_mps = 0.0
        command.steering_angle_rad = 0.0
        command.enable = True
        command.brake = False
        command.clear_fault = False
        command.emergency_stop = False
        self.command_pub.publish(command)

        self._publish_diagnostics(stamp, output, target_speed_mps=0.0, steering_command_rad=0.0)

    def _publish_control(self, stamp, output) -> None:
        command = AckermannChassisCommand()
        command.header.stamp = stamp
        command.speed_mps = float(output.speed_mps)
        command.steering_angle_rad = float(output.steering_angle_rad)
        command.enable = True
        command.brake = bool(output.brake)
        command.clear_fault = False
        command.emergency_stop = False
        self.command_pub.publish(command)

        self._publish_diagnostics(
            stamp,
            output,
            target_speed_mps=float(output.target_speed_mps),
            steering_command_rad=float(output.steering_angle_rad),
        )

    def _publish_diagnostics(
        self,
        stamp,
        output,
        *,
        target_speed_mps: float,
        steering_command_rad: float,
    ) -> None:
        diagnostics = PathTrackingDiagnostics()
        diagnostics.header.stamp = stamp
        diagnostics.case_name = self.case_name
        diagnostics.target_index = int(output.target_index)
        diagnostics.lookahead_m = float(output.lookahead_m)
        diagnostics.lateral_error_m = float(output.lateral_error_m)
        diagnostics.heading_error_rad = float(output.heading_error_rad)
        diagnostics.target_speed_mps = target_speed_mps
        diagnostics.actual_speed_mps = float(self.actual_speed_mps)
        diagnostics.steering_command_rad = steering_command_rad
        diagnostics.steering_limited = bool(output.steering_limited)
        diagnostics.speed_limited = bool(output.speed_limited)
        diagnostics.stop_commanded = bool(output.stop_commanded)
        diagnostics.remaining_distance_m = float(output.remaining_distance_m)
        self.diagnostics_pub.publish(diagnostics)


def main() -> None:
    rclpy.init()
    node = PurePursuitTracker()
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
