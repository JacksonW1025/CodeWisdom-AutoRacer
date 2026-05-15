#!/usr/bin/env python3
"""Offline and live checks for phase-1 Ackermann chassis acceptance.

The default mode is offline and never opens the serial port or publishes
movement commands. Live mode only inspects ROS topics that are already running.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
COMMAND_ACKERMANN = 0x01
COMMAND_SIZE = 11
TELEMETRY_SIZE = 24

FLAG_ENABLE = 1 << 0
FLAG_BRAKE = 1 << 1
FLAG_CLEAR_FAULT = 1 << 2
FLAG_EMERGENCY_STOP = 1 << 7

TELEMETRY_AUTO_ENABLED = 1 << 0
TELEMETRY_RC_OVERRIDE_ACTIVE = 1 << 1
TELEMETRY_ESTOP_ACTIVE = 1 << 2
TELEMETRY_COMMAND_TIMEOUT = 1 << 3
TELEMETRY_BRAKE_ACTIVE = 1 << 4

STATUS_STEERING_FEEDBACK_VALID = 1 << 8
STATUS_STEERING_IS_MEASURED = 1 << 9


@dataclass(frozen=True)
class CommandLimits:
    max_auto_speed_mps: float = 0.50
    max_reverse_speed_mps: float = 0.30
    max_steering_angle_rad: float = 0.393


@dataclass
class CheckResult:
    name: str
    passed: bool
    detail: str


class CheckFailure(AssertionError):
    pass


def checksum(data: Iterable[int]) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value & 0xFF


def i16_to_bytes(value: int) -> list[int]:
    if value < 0:
        value = (1 << 16) + value
    return [(value >> 8) & 0xFF, value & 0xFF]


def i32_to_bytes(value: int) -> list[int]:
    if value < 0:
        value = (1 << 32) + value
    return [
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF,
    ]


def u16_to_bytes(value: int) -> list[int]:
    return [(value >> 8) & 0xFF, value & 0xFF]


def u32_to_bytes(value: int) -> list[int]:
    return [
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF,
    ]


def read_i16(data: list[int], offset: int) -> int:
    value = (data[offset] << 8) | data[offset + 1]
    return value - (1 << 16) if value & 0x8000 else value


def read_u16(data: list[int], offset: int) -> int:
    return (data[offset] << 8) | data[offset + 1]


def read_i32(data: list[int], offset: int) -> int:
    value = (
        (data[offset] << 24)
        | (data[offset + 1] << 16)
        | (data[offset + 2] << 8)
        | data[offset + 3]
    )
    return value - (1 << 32) if value & 0x80000000 else value


def read_u32(data: list[int], offset: int) -> int:
    return (
        (data[offset] << 24)
        | (data[offset + 1] << 16)
        | (data[offset + 2] << 8)
        | data[offset + 3]
    )


def to_i16_saturated(value: float) -> int:
    rounded = int(round(value))
    return max(-(1 << 15), min((1 << 15) - 1, rounded))


def build_command_frame(
    speed_mps: float,
    steering_angle_rad: float,
    *,
    enable: bool,
    brake: bool = False,
    clear_fault: bool = False,
    emergency_stop: bool = False,
    limits: CommandLimits = CommandLimits(),
) -> list[int]:
    speed = max(-limits.max_reverse_speed_mps, min(limits.max_auto_speed_mps, speed_mps))
    steering = max(
        -limits.max_steering_angle_rad,
        min(limits.max_steering_angle_rad, steering_angle_rad),
    )

    flags = 0
    if enable:
        flags |= FLAG_ENABLE
    if brake:
        flags |= FLAG_BRAKE
        speed = 0.0
    if clear_fault:
        flags |= FLAG_CLEAR_FAULT
    if emergency_stop:
        flags |= FLAG_EMERGENCY_STOP
        speed = 0.0
    if not enable:
        speed = 0.0

    frame = [0] * COMMAND_SIZE
    frame[0] = FRAME_HEADER
    frame[1] = COMMAND_ACKERMANN
    frame[2] = flags
    frame[3:5] = i16_to_bytes(to_i16_saturated(speed * 1000.0))
    frame[5:7] = i16_to_bytes(to_i16_saturated(steering * 1000.0))
    frame[7:9] = [0, 0]
    frame[9] = checksum(frame[:9])
    frame[10] = FRAME_TAIL
    return frame


def make_telemetry_frame(
    *,
    status_flags: int,
    seq: int,
    hall_delta_count: int,
    speed_mmps: int,
    steering_mrad: int,
    yaw_rate_mradps: int,
    battery_mv: int,
    dt_ms: int,
    status_bits: int,
) -> list[int]:
    frame = [0] * TELEMETRY_SIZE
    frame[0] = FRAME_HEADER
    frame[1] = status_flags & 0xFF
    frame[2] = seq & 0xFF
    frame[3:7] = i32_to_bytes(hall_delta_count)
    frame[7:9] = i16_to_bytes(speed_mmps)
    frame[9:11] = i16_to_bytes(steering_mrad)
    frame[11:13] = i16_to_bytes(yaw_rate_mradps)
    frame[13:15] = u16_to_bytes(battery_mv)
    frame[15:17] = u16_to_bytes(dt_ms)
    frame[17:21] = u32_to_bytes(status_bits)
    frame[21] = 0
    frame[22] = checksum(frame[:22])
    frame[23] = FRAME_TAIL
    return frame


def parse_telemetry_frame(frame: list[int], counts_per_meter: float) -> dict[str, object]:
    if len(frame) != TELEMETRY_SIZE:
        raise CheckFailure(f"expected {TELEMETRY_SIZE} bytes, got {len(frame)}")
    if frame[0] != FRAME_HEADER or frame[23] != FRAME_TAIL:
        raise CheckFailure("bad telemetry header or tail")
    if checksum(frame[:22]) != frame[22]:
        raise CheckFailure("bad telemetry BCC")

    status_flags = frame[1]
    status_bits = read_u32(frame, 17)
    hall_delta_count = read_i32(frame, 3)

    return {
        "seq": frame[2],
        "hall_delta_count": hall_delta_count,
        "delta_s_m": hall_delta_count / counts_per_meter if counts_per_meter > 0.0 else 0.0,
        "actual_speed_mps": read_i16(frame, 7) / 1000.0,
        "steering_angle_rad": read_i16(frame, 9) / 1000.0,
        "steering_feedback_valid": bool(status_bits & STATUS_STEERING_FEEDBACK_VALID),
        "steering_is_measured": bool(status_bits & STATUS_STEERING_IS_MEASURED),
        "estimated_yaw_rate_radps": read_i16(frame, 11) / 1000.0,
        "battery_voltage": read_u16(frame, 13) / 1000.0,
        "dt_ms": read_u16(frame, 15),
        "auto_enabled": bool(status_flags & TELEMETRY_AUTO_ENABLED),
        "rc_override_active": bool(status_flags & TELEMETRY_RC_OVERRIDE_ACTIVE),
        "estop_active": bool(status_flags & TELEMETRY_ESTOP_ACTIVE),
        "brake_active": bool(status_flags & TELEMETRY_BRAKE_ACTIVE),
        "command_timeout": bool(status_flags & TELEMETRY_COMMAND_TIMEOUT),
        "status_bits": status_bits,
    }


def should_publish_wheel_odom(counts_per_meter: float, steering_feedback_valid: bool) -> bool:
    return counts_per_meter > 0.0 and steering_feedback_valid


def require(condition: bool, detail: str) -> None:
    if not condition:
        raise CheckFailure(detail)


def check_protocol_downlink() -> None:
    frame = build_command_frame(0.2, 0.1, enable=True)
    require(len(frame) == COMMAND_SIZE, "command frame length mismatch")
    require(frame[0] == FRAME_HEADER and frame[10] == FRAME_TAIL, "command header/tail mismatch")
    require(frame[1] == COMMAND_ACKERMANN, "command id must be 0x01")
    require(frame[2] == FLAG_ENABLE, "enable flag mismatch")
    require(read_i16(frame, 3) == 200, "speed_mps must encode as speed_mmps")
    require(read_i16(frame, 5) == 100, "steering_angle_rad must encode as steering_mrad")
    require(read_i16(frame, 7) == 0, "reserved field must be zero")
    require(frame[9] == checksum(frame[:9]), "command BCC mismatch")

    disabled = build_command_frame(0.2, 0.1, enable=False)
    require(read_i16(disabled, 3) == 0, "disabled command must force speed to zero")

    brake = build_command_frame(0.2, 0.1, enable=True, brake=True)
    require(brake[2] == (FLAG_ENABLE | FLAG_BRAKE), "brake flag mismatch")
    require(read_i16(brake, 3) == 0, "brake must force speed to zero")

    estop = build_command_frame(0.2, 0.1, enable=True, emergency_stop=True)
    require(estop[2] == (FLAG_ENABLE | FLAG_EMERGENCY_STOP), "estop flag mismatch")
    require(read_i16(estop, 3) == 0, "estop must force speed to zero")

    saturated = build_command_frame(9.0, 9.0, enable=True)
    require(read_i16(saturated, 3) == 500, "forward speed clamp mismatch")
    require(read_i16(saturated, 5) == 393, "steering clamp mismatch")

    reverse = build_command_frame(-9.0, -9.0, enable=True)
    require(read_i16(reverse, 3) == -300, "reverse speed clamp mismatch")
    require(read_i16(reverse, 5) == -393, "negative steering clamp mismatch")


def check_telemetry_uplink() -> None:
    status_flags = (
        TELEMETRY_AUTO_ENABLED
        | TELEMETRY_RC_OVERRIDE_ACTIVE
        | TELEMETRY_ESTOP_ACTIVE
        | TELEMETRY_COMMAND_TIMEOUT
        | TELEMETRY_BRAKE_ACTIVE
    )
    status_bits = STATUS_STEERING_FEEDBACK_VALID | STATUS_STEERING_IS_MEASURED | (1 << 11)
    frame = make_telemetry_frame(
        status_flags=status_flags,
        seq=42,
        hall_delta_count=-27,
        speed_mmps=-250,
        steering_mrad=123,
        yaw_rate_mradps=-45,
        battery_mv=7400,
        dt_ms=20,
        status_bits=status_bits,
    )
    state = parse_telemetry_frame(frame, counts_per_meter=13.5)
    require(state["seq"] == 42, "seq decode mismatch")
    require(state["hall_delta_count"] == -27, "hall_delta_count decode mismatch")
    require(math.isclose(state["delta_s_m"], -2.0), "delta_s_m decode mismatch")
    require(math.isclose(state["actual_speed_mps"], -0.25), "speed decode mismatch")
    require(math.isclose(state["steering_angle_rad"], 0.123), "steering decode mismatch")
    require(math.isclose(state["estimated_yaw_rate_radps"], -0.045), "yaw-rate decode mismatch")
    require(math.isclose(state["battery_voltage"], 7.4), "battery decode mismatch")
    require(state["dt_ms"] == 20, "dt_ms decode mismatch")
    require(state["auto_enabled"], "AUTO_ENABLED status flag missing")
    require(state["rc_override_active"], "RC_OVERRIDE_ACTIVE status flag missing")
    require(state["estop_active"], "ESTOP_ACTIVE status flag missing")
    require(state["command_timeout"], "COMMAND_TIMEOUT status flag missing")
    require(state["brake_active"], "BRAKE_ACTIVE status flag missing")
    require(state["steering_feedback_valid"], "STEERING_FEEDBACK_VALID status bit missing")
    require(state["steering_is_measured"], "STEERING_IS_MEASURED status bit missing")

    bad_bcc = list(frame)
    bad_bcc[22] ^= 0x01
    try:
        parse_telemetry_frame(bad_bcc, counts_per_meter=13.5)
    except CheckFailure:
        pass
    else:
        raise CheckFailure("bad telemetry BCC was accepted")

    bad_header = list(frame)
    bad_header[0] = 0
    try:
        parse_telemetry_frame(bad_header, counts_per_meter=13.5)
    except CheckFailure:
        pass
    else:
        raise CheckFailure("bad telemetry header was accepted")


def check_counts_per_meter_gate(repo_root: Path) -> None:
    source = repo_root / "src" / "turn_on_autoracer_robot" / "src" / "ackermann_chassis_bridge.cpp"
    launch = repo_root / "src" / "turn_on_autoracer_robot" / "launch" / "ackermann_chassis.launch.py"
    config = repo_root / "src" / "turn_on_autoracer_robot" / "config" / "ackermann_chassis.yaml"

    source_text = source.read_text(encoding="utf-8")
    launch_text = launch.read_text(encoding="utf-8")
    config_text = config.read_text(encoding="utf-8")

    require("counts_per_meter_ <= 0.0" in source_text, "source lacks wheel_odom counts gate")
    require("Skipping wheel_odom because steering feedback is invalid" in source_text, "source lacks steering-valid gate")
    require("default_value='0.0'" in launch_text, "launch counts_per_meter default must be safe 0.0")
    require("'use_ekf'" in launch_text and "default_value='false'" in launch_text, "launch use_ekf default must be explicit false")
    require("counts_per_meter:" in config_text, "config lacks counts_per_meter parameter")

    require(not should_publish_wheel_odom(0.0, True), "counts_per_meter=0 must block wheel_odom")
    require(not should_publish_wheel_odom(-1.0, True), "counts_per_meter<0 must block wheel_odom")
    require(not should_publish_wheel_odom(13.5, False), "invalid steering feedback must block wheel_odom")
    require(should_publish_wheel_odom(13.5, True), "valid counts and steering feedback should allow wheel_odom")


def check_launch_contract(repo_root: Path) -> None:
    launch = repo_root / "src" / "turn_on_autoracer_robot" / "launch" / "ackermann_chassis.launch.py"
    ekf_config = repo_root / "src" / "turn_on_autoracer_robot" / "config" / "ackermann_ekf.yaml"
    package_xml = repo_root / "src" / "turn_on_autoracer_robot" / "package.xml"
    fixed_launch = repo_root / "src" / "autoracer_bringup" / "launch" / "stage1_chassis.launch.py"
    launch_text = launch.read_text(encoding="utf-8")
    ekf_text = ekf_config.read_text(encoding="utf-8")
    package_text = package_xml.read_text(encoding="utf-8")
    fixed_launch_text = fixed_launch.read_text(encoding="utf-8")

    require("ackermann_chassis_bridge" in launch_text, "launch must start ackermann_chassis_bridge")
    require("robot_localization" in launch_text and "ekf_node" in launch_text, "launch must define EKF node")
    require("IfCondition(LaunchConfiguration('use_ekf'))" in launch_text, "EKF must be gated by use_ekf")
    require("('/odometry/filtered', '/odom')" in launch_text, "EKF output must remap to canonical /odom")
    require("serial_baud_rate" in launch_text and "115200" in launch_text, "launch must expose 115200 serial baud")
    require("wheel_odom" in ekf_text and "/imu/data" in ekf_text, "EKF config must consume wheel_odom and /imu/data")
    require("<exec_depend>robot_localization</exec_depend>" in package_text, "package must declare robot_localization runtime dependency")
    require("ackermann_chassis.launch.py" in fixed_launch_text, "stage1 fixed launch must include Ackermann chassis launch")
    require("default_value='true'" in fixed_launch_text and "'use_ekf'" in fixed_launch_text,
            "stage1 fixed launch must default use_ekf true")


def run_offline_checks(repo_root: Path) -> list[CheckResult]:
    checks = [
        ("protocol_downlink", lambda: check_protocol_downlink()),
        ("telemetry_uplink", lambda: check_telemetry_uplink()),
        ("wheel_odom_gate", lambda: check_counts_per_meter_gate(repo_root)),
        ("launch_contract", lambda: check_launch_contract(repo_root)),
    ]
    return run_checks(checks)


def run_checks(checks: list[tuple[str, object]]) -> list[CheckResult]:
    results: list[CheckResult] = []
    for name, check in checks:
        try:
            check()
        except Exception as exc:  # noqa: BLE001 - command-line report should include all failures.
            results.append(CheckResult(name, False, str(exc)))
        else:
            results.append(CheckResult(name, True, "ok"))
    return results


def ros2_output(args: list[str], timeout: float) -> str:
    completed = subprocess.run(
        ["ros2", *args],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=timeout,
    )
    if completed.returncode != 0:
        raise CheckFailure(f"ros2 {' '.join(args)} failed: {completed.stdout.strip()}")
    return completed.stdout


def check_live_topics(timeout: float, require_odom: bool) -> None:
    topics = set(line.strip() for line in ros2_output(["topic", "list"], timeout).splitlines())
    required = {"/ackermann_cmd", "/chassis_state"}
    if require_odom:
        required.update({"/wheel_odom", "/odom", "/imu/data"})
    missing = sorted(required - topics)
    require(not missing, f"missing required topics: {', '.join(missing)}")

    expected_types = {
        "/ackermann_cmd": "autoracer_interfaces/msg/AckermannChassisCommand",
        "/chassis_state": "autoracer_interfaces/msg/ChassisState",
        "/wheel_odom": "nav_msgs/msg/Odometry",
        "/odom": "nav_msgs/msg/Odometry",
        "/imu/data": "sensor_msgs/msg/Imu",
    }
    for topic in sorted(required):
        info = ros2_output(["topic", "info", topic], timeout)
        expected = expected_types[topic]
        require(expected in info, f"{topic} type mismatch; expected {expected}")


def run_live_checks(timeout: float, require_odom: bool) -> list[CheckResult]:
    return run_checks([("live_topics", lambda: check_live_topics(timeout, require_odom))])


def print_results(results: list[CheckResult]) -> None:
    for result in results:
        status = "PASS" if result.passed else "FAIL"
        print(f"{status} {result.name}: {result.detail}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[2],
        help="CodeWisdom-AutoRacer repository root",
    )
    parser.add_argument(
        "--mode",
        choices=("offline", "live", "all"),
        default="offline",
        help="offline checks do not require ROS nodes; live checks only inspect running topics",
    )
    parser.add_argument(
        "--require-odom",
        action="store_true",
        help="in live mode, require /wheel_odom, /odom, and /imu/data in addition to bridge topics",
    )
    parser.add_argument("--timeout", type=float, default=5.0, help="ros2 command timeout in seconds")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    results: list[CheckResult] = []

    if args.mode in ("offline", "all"):
        results.extend(run_offline_checks(repo_root))
    if args.mode in ("live", "all"):
        results.extend(run_live_checks(args.timeout, args.require_odom))

    print_results(results)
    return 0 if all(result.passed for result in results) else 1


if __name__ == "__main__":
    sys.exit(main())
