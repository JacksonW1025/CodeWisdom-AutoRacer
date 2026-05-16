#!/usr/bin/env python3
"""Convert Nav2 internal Twist commands into formal Ackermann chassis commands."""

from __future__ import annotations

import math
from dataclasses import dataclass


STOP_REASON_NONE = "none"
STOP_REASON_ZERO = "zero_command"
STOP_REASON_INVALID = "invalid_input"
STOP_REASON_TIMEOUT = "input_timeout"
STOP_REASON_INFEASIBLE_SPIN = "infeasible_spin"
STOP_REASON_REVERSE_DISABLED = "reverse_disabled"

STOP_REASONS = {
    STOP_REASON_NONE,
    STOP_REASON_ZERO,
    STOP_REASON_INVALID,
    STOP_REASON_TIMEOUT,
    STOP_REASON_INFEASIBLE_SPIN,
    STOP_REASON_REVERSE_DISABLED,
}


@dataclass(frozen=True)
class AdapterConfig:
    wheelbase_m: float = 0.60
    max_steering_angle_rad: float = 0.262
    max_auto_speed_mps: float = 1.00
    max_reverse_speed_mps: float = 0.60
    min_turn_speed_mps: float = 0.05
    angular_deadband_radps: float = 0.02
    allow_reverse: bool = False
    brake_on_stop: bool = True
    enable_on_command: bool = True


@dataclass(frozen=True)
class AdapterOutput:
    speed_mps: float
    steering_angle_rad: float
    enable: bool
    brake: bool
    clear_fault: bool
    emergency_stop: bool
    speed_limited: bool
    steering_limited: bool
    reverse: bool
    infeasible_spin: bool
    stop_reason: str


def clamp(value: float, lower: float, upper: float) -> tuple[float, bool]:
    clamped = max(lower, min(upper, value))
    return clamped, not math.isclose(clamped, value, abs_tol=1.0e-9)


def stop_output(config: AdapterConfig, reason: str, *, reverse: bool = False, infeasible_spin: bool = False) -> AdapterOutput:
    if reason not in STOP_REASONS:
        raise ValueError(f"unknown stop_reason {reason!r}")
    return AdapterOutput(
        speed_mps=0.0,
        steering_angle_rad=0.0,
        enable=config.enable_on_command,
        brake=config.brake_on_stop,
        clear_fault=False,
        emergency_stop=False,
        speed_limited=False,
        steering_limited=False,
        reverse=reverse,
        infeasible_spin=infeasible_spin,
        stop_reason=reason,
    )


def compute_ackermann(vx: float, wz: float, config: AdapterConfig) -> AdapterOutput:
    if not math.isfinite(vx) or not math.isfinite(wz):
        return stop_output(config, STOP_REASON_INVALID)

    reverse = vx < 0.0
    if vx == 0.0 and abs(wz) <= config.angular_deadband_radps:
        return stop_output(config, STOP_REASON_ZERO)

    if abs(vx) < config.min_turn_speed_mps and abs(wz) > config.angular_deadband_radps:
        return stop_output(config, STOP_REASON_INFEASIBLE_SPIN, reverse=reverse, infeasible_spin=True)

    if reverse and not config.allow_reverse:
        return stop_output(config, STOP_REASON_REVERSE_DISABLED, reverse=True)

    if reverse:
        speed, speed_limited = clamp(vx, -config.max_reverse_speed_mps, 0.0)
    else:
        speed, speed_limited = clamp(vx, 0.0, config.max_auto_speed_mps)

    if abs(wz) <= config.angular_deadband_radps:
        steering = 0.0
        steering_limited = False
    else:
        steering_raw = math.atan(config.wheelbase_m * wz / vx)
        steering, steering_limited = clamp(
            steering_raw,
            -config.max_steering_angle_rad,
            config.max_steering_angle_rad,
        )

    return AdapterOutput(
        speed_mps=speed,
        steering_angle_rad=steering,
        enable=config.enable_on_command,
        brake=False,
        clear_fault=False,
        emergency_stop=False,
        speed_limited=speed_limited,
        steering_limited=steering_limited,
        reverse=reverse,
        infeasible_spin=False,
        stop_reason=STOP_REASON_NONE,
    )


class TwistToAckermannNode:
    def __init__(self) -> None:
        import rclpy
        from autoracer_interfaces.msg import AckermannChassisCommand
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import Twist
        from rclpy.node import Node

        class _Node(Node):
            pass

        self._rclpy = rclpy
        self._AckermannChassisCommand = AckermannChassisCommand
        self._DiagnosticArray = DiagnosticArray
        self._Twist = Twist
        self.node = _Node("twist_to_ackermann")

        self.node.declare_parameter("input_topic", "/safe_nav2_cmd_vel")
        self.node.declare_parameter("output_topic", "/ackermann_cmd")
        self.node.declare_parameter("diagnostics_topic", "/twist_to_ackermann/diagnostics")
        self.node.declare_parameter("wheelbase_m", 0.60)
        self.node.declare_parameter("max_steering_angle_rad", 0.262)
        self.node.declare_parameter("max_auto_speed_mps", 1.00)
        self.node.declare_parameter("max_reverse_speed_mps", 0.60)
        self.node.declare_parameter("min_turn_speed_mps", 0.05)
        self.node.declare_parameter("angular_deadband_radps", 0.02)
        self.node.declare_parameter("allow_reverse", False)
        self.node.declare_parameter("input_timeout_sec", 0.50)
        self.node.declare_parameter("brake_on_stop", True)
        self.node.declare_parameter("enable_on_command", True)
        self.node.declare_parameter("publish_timeout_stop", True)
        self.node.declare_parameter("diagnostics_rate_hz", 10.0)

        self.config = self._read_config()
        self.input_timeout_sec = float(self.node.get_parameter("input_timeout_sec").value)
        self.publish_timeout_stop = bool(self.node.get_parameter("publish_timeout_stop").value)
        diagnostics_rate_hz = max(1.0, float(self.node.get_parameter("diagnostics_rate_hz").value))

        input_topic = str(self.node.get_parameter("input_topic").value)
        output_topic = str(self.node.get_parameter("output_topic").value)
        diagnostics_topic = str(self.node.get_parameter("diagnostics_topic").value)

        self.command_pub = self.node.create_publisher(AckermannChassisCommand, output_topic, 10)
        self.diagnostics_pub = self.node.create_publisher(DiagnosticArray, diagnostics_topic, 10)
        self.subscription = self.node.create_subscription(Twist, input_topic, self._on_twist, 10)
        self.timer = self.node.create_timer(1.0 / diagnostics_rate_hz, self._on_timer)

        self.last_input_time = None
        self.last_output = stop_output(self.config, STOP_REASON_ZERO)
        self.timeout_stop_published = False

    def _read_config(self) -> AdapterConfig:
        return AdapterConfig(
            wheelbase_m=float(self.node.get_parameter("wheelbase_m").value),
            max_steering_angle_rad=float(self.node.get_parameter("max_steering_angle_rad").value),
            max_auto_speed_mps=float(self.node.get_parameter("max_auto_speed_mps").value),
            max_reverse_speed_mps=float(self.node.get_parameter("max_reverse_speed_mps").value),
            min_turn_speed_mps=float(self.node.get_parameter("min_turn_speed_mps").value),
            angular_deadband_radps=float(self.node.get_parameter("angular_deadband_radps").value),
            allow_reverse=bool(self.node.get_parameter("allow_reverse").value),
            brake_on_stop=bool(self.node.get_parameter("brake_on_stop").value),
            enable_on_command=bool(self.node.get_parameter("enable_on_command").value),
        )

    def _on_twist(self, msg) -> None:
        self.last_input_time = self.node.get_clock().now()
        self.timeout_stop_published = False
        self.last_output = compute_ackermann(float(msg.linear.x), float(msg.angular.z), self.config)
        self._publish_command(self.last_output)
        self._publish_diagnostics(float(msg.linear.x), float(msg.angular.z), self.last_output)

    def _on_timer(self) -> None:
        now = self.node.get_clock().now()
        if self.last_input_time is None:
            timed_out = True
        else:
            timed_out = (now - self.last_input_time).nanoseconds * 1.0e-9 > self.input_timeout_sec

        if timed_out:
            self.last_output = stop_output(self.config, STOP_REASON_TIMEOUT)
            if self.publish_timeout_stop or not self.timeout_stop_published:
                self._publish_command(self.last_output)
                self.timeout_stop_published = True
            self._publish_diagnostics(0.0, 0.0, self.last_output)
        else:
            self._publish_diagnostics(0.0, 0.0, self.last_output)

    def _publish_command(self, output: AdapterOutput) -> None:
        msg = self._AckermannChassisCommand()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.speed_mps = float(output.speed_mps)
        msg.steering_angle_rad = float(output.steering_angle_rad)
        msg.enable = bool(output.enable)
        msg.brake = bool(output.brake)
        msg.clear_fault = bool(output.clear_fault)
        msg.emergency_stop = bool(output.emergency_stop)
        self.command_pub.publish(msg)

    def _publish_diagnostics(self, input_vx: float, input_wz: float, output: AdapterOutput) -> None:
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

        status = DiagnosticStatus()
        status.name = "twist_to_ackermann"
        status.hardware_id = "autoracer_nav2_adapter"
        status.message = output.stop_reason
        status.level = DiagnosticStatus.OK if output.stop_reason in (STOP_REASON_NONE, STOP_REASON_ZERO) else DiagnosticStatus.WARN
        status.values = [
            KeyValue(key="input_vx_mps", value=f"{input_vx:.6f}"),
            KeyValue(key="input_wz_radps", value=f"{input_wz:.6f}"),
            KeyValue(key="output_speed_mps", value=f"{output.speed_mps:.6f}"),
            KeyValue(key="output_steering_angle_rad", value=f"{output.steering_angle_rad:.6f}"),
            KeyValue(key="speed_limited", value=str(output.speed_limited).lower()),
            KeyValue(key="steering_limited", value=str(output.steering_limited).lower()),
            KeyValue(key="reverse", value=str(output.reverse).lower()),
            KeyValue(key="infeasible_spin", value=str(output.infeasible_spin).lower()),
            KeyValue(key="stop_reason", value=output.stop_reason),
        ]

        msg = DiagnosticArray()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.status.append(status)
        self.diagnostics_pub.publish(msg)


def main() -> None:
    import rclpy

    rclpy.init()
    adapter = TwistToAckermannNode()
    try:
        rclpy.spin(adapter.node)
    finally:
        adapter.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
