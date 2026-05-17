#!/usr/bin/env python3
"""Offline contract checks for phase-4 Nav2 Ackermann navigation."""

from __future__ import annotations

import argparse
import importlib.util
import math
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass
class CheckResult:
    name: str
    passed: bool
    detail: str


class ContractFailure(AssertionError):
    pass


def require(condition: bool, detail: str) -> None:
    if not condition:
        raise ContractFailure(detail)


def require_text(text: str, needle: str, detail: str) -> None:
    require(needle in text, detail)


def load_adapter(repo_root: Path):
    path = repo_root / "src" / "autoracer_robot_nav2" / "scripts" / "twist_to_ackermann.py"
    spec = importlib.util.spec_from_file_location("twist_to_ackermann_contract", path)
    require(spec is not None and spec.loader is not None, "could not load twist_to_ackermann module spec")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def check_stage4_launch(repo_root: Path) -> None:
    launch = repo_root / "src" / "autoracer_bringup" / "launch" / "stage4_navigation.launch.py"
    text = launch.read_text(encoding="utf-8")

    required = {
        "stage1_chassis.launch.py": "stage4 launch must include phase-1 chassis entry",
        "imu_spec_msg.launch.py": "stage4 launch must include IMU raw driver",
        "imu_filter_madgwick_node": "stage4 launch must include Madgwick /imu/data filter",
        "robot_description.launch.py": "stage4 launch must include URDF/TF launch",
        "lslidar_cx_launch.py": "stage4 launch must include LiDAR launch",
        "stage4_nav2_params.yaml": "stage4 launch must default to stable stage4 params",
        "nav2_bringup": "stage4 launch must include Nav2 bringup",
        "localization_launch.py": "stage4 launch must include localization",
        "navigation_launch.py": "stage4 launch must include navigation",
        "SetRemap(src='cmd_vel', dst='/nav2_cmd_vel')": "Nav2 cmd_vel must be isolated to /nav2_cmd_vel",
        "nav2_collision_monitor": "stage4 launch must start Collision Monitor",
        "lifecycle_manager_collision_monitor": "Collision Monitor must be lifecycle managed",
        "twist_to_ackermann": "stage4 launch must start adapter",
        "'input_topic': '/safe_nav2_cmd_vel'": "adapter must consume /safe_nav2_cmd_vel",
        "'output_topic': '/ackermann_cmd'": "adapter must publish /ackermann_cmd",
        "'diagnostics_topic': '/twist_to_ackermann/diagnostics'": "adapter diagnostics topic mismatch",
        "start_chassis": "stage4 launch must allow non-hardware dry launch by disabling chassis",
        "start_imu": "stage4 launch must allow IMU dry-launch control",
        "start_robot_description": "stage4 launch must allow URDF/TF dry-launch control",
        "start_lidar": "stage4 launch must allow non-hardware dry launch by disabling LiDAR",
        "start_nav2": "stage4 launch must allow local dry launch parsing when Nav2 is not installed",
        "validate_stage4_required_inputs": "stage4 launch must validate required live inputs",
        "counts_per_meter must be > 0": "stage4 launch must reject uncalibrated counts_per_meter",
        "stage4 map must be set": "stage4 launch must require an explicit saved map",
        "stage4 map file does not exist": "stage4 launch must check map path existence",
    }
    for needle, detail in required.items():
        require_text(text, needle, detail)


def check_robot_description_tf(repo_root: Path) -> None:
    launch = repo_root / "src" / "autoracer_robot_urdf" / "launch" / "robot_description.launch.py"
    text = launch.read_text(encoding="utf-8")

    required = {
        "base_footprint_to_base_link": "robot_description must publish base_footprint -> base_link",
        "'--z', '0.1175'": "base_link must be anchored at axle height above base_footprint",
        "'--frame-id', 'base_footprint'": "base transform parent must be base_footprint",
        "'--child-frame-id', 'base_link'": "base transform child must be base_link",
        "robot_state_publisher": "robot_description must publish URDF fixed sensor TF",
    }
    for needle, detail in required.items():
        require_text(text, needle, detail)


def check_nav2_params(repo_root: Path) -> None:
    params = repo_root / "src" / "autoracer_robot_nav2" / "param" / "stage4_nav2_params.yaml"
    text = params.read_text(encoding="utf-8")

    required = {
        'plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"': "controller must be RPP",
        "allow_reversing: false": "stage4A must disable reversing",
        "use_rotate_to_heading: false": "RPP rotate-to-heading must be disabled for Ackermann",
        'plugin: "nav2_smac_planner/SmacPlannerHybrid"': "planner must be Smac Hybrid-A*",
        'motion_model_for_search: "DUBIN"': "stage4A planner must be forward-only DUBIN",
        "minimum_turning_radius: 2.24": "minimum turning radius must be 2.24 m",
        "cmd_vel_in_topic: \"/nav2_cmd_vel\"": "Collision Monitor input topic mismatch",
        "cmd_vel_out_topic: \"/safe_nav2_cmd_vel\"": "Collision Monitor output topic mismatch",
        "enable_stamped_cmd_vel: false": "Collision Monitor must use Twist for Humble adapter chain",
        "front_stop": "front stop zone missing",
        "front_slowdown": "front slowdown zone missing",
        "rear_stop": "rear stop zone missing",
        "rear_slowdown": "rear slowdown zone missing",
        "left_side_stop": "left side stop zone missing",
        "right_side_stop": "right side stop zone missing",
        "observation_sources: [\"scan\"]": "Collision Monitor must use /scan observation source",
        "topic: \"/scan\"": "Collision Monitor scan source must use /scan",
        "max_points: 3": "Humble Collision Monitor compatibility must include max_points",
    }
    for needle, detail in required.items():
        require_text(text, needle, detail)

    require("nav2_mppi_controller::MPPIController" not in text, "stage4 stable params must not use MPPI")
    require('action_type: "limit"' not in text, "ROS2 Humble baseline must not use Collision Monitor limit action")


def check_adapter_math(repo_root: Path) -> None:
    adapter = load_adapter(repo_root)
    config = adapter.AdapterConfig()

    normal = adapter.compute_ackermann(0.30, 0.10, config)
    require(math.isclose(normal.speed_mps, 0.30), "normal speed mismatch")
    require(normal.stop_reason == adapter.STOP_REASON_NONE, "normal command should not stop")
    require(not normal.brake, "normal command must not brake")

    limited = adapter.compute_ackermann(2.0, 0.0, config)
    require(math.isclose(limited.speed_mps, 1.0), "forward speed clamp mismatch")
    require(limited.speed_limited, "forward speed clamp must set speed_limited")

    steering = adapter.compute_ackermann(0.10, 5.0, config)
    require(math.isclose(steering.steering_angle_rad, 0.262), "steering clamp mismatch")
    require(steering.steering_limited, "steering clamp must set steering_limited")

    zero = adapter.compute_ackermann(0.0, 0.0, config)
    require(zero.stop_reason == adapter.STOP_REASON_ZERO and zero.brake, "zero command must brake with zero_command reason")

    spin = adapter.compute_ackermann(0.0, 0.30, config)
    require(spin.stop_reason == adapter.STOP_REASON_INFEASIBLE_SPIN, "spin must be rejected")
    require(spin.infeasible_spin, "spin rejection must set infeasible_spin")

    invalid = adapter.compute_ackermann(float("nan"), 0.0, config)
    require(invalid.stop_reason == adapter.STOP_REASON_INVALID, "NaN input must stop")

    reverse_disabled = adapter.compute_ackermann(-0.20, 0.0, config)
    require(reverse_disabled.stop_reason == adapter.STOP_REASON_REVERSE_DISABLED, "reverse must be disabled in stage4A")
    require(reverse_disabled.reverse, "reverse_disabled must set reverse flag")

    reverse_config = adapter.AdapterConfig(allow_reverse=True)
    reverse_limited = adapter.compute_ackermann(-0.80, 0.0, reverse_config)
    require(math.isclose(reverse_limited.speed_mps, -0.60), "reverse clamp mismatch")
    require(reverse_limited.speed_limited, "reverse clamp must set speed_limited")


def check_adapter_diagnostics_contract(repo_root: Path) -> None:
    adapter = repo_root / "src" / "autoracer_robot_nav2" / "scripts" / "twist_to_ackermann.py"
    text = adapter.read_text(encoding="utf-8")
    for field in (
        "input_vx_mps",
        "input_wz_radps",
        "output_speed_mps",
        "output_steering_angle_rad",
        "speed_limited",
        "steering_limited",
        "reverse",
        "infeasible_spin",
        "stop_reason",
    ):
        require_text(text, f'key="{field}"', f"diagnostics must include {field}")
    for reason in (
        "none",
        "zero_command",
        "invalid_input",
        "input_timeout",
        "infeasible_spin",
        "reverse_disabled",
    ):
        require_text(text, reason, f"adapter must define stop_reason={reason}")


def run_checks(repo_root: Path) -> list[CheckResult]:
    checks = [
        ("stage4_launch", lambda: check_stage4_launch(repo_root)),
        ("robot_description_tf", lambda: check_robot_description_tf(repo_root)),
        ("nav2_params", lambda: check_nav2_params(repo_root)),
        ("adapter_math", lambda: check_adapter_math(repo_root)),
        ("adapter_diagnostics_contract", lambda: check_adapter_diagnostics_contract(repo_root)),
    ]
    results: list[CheckResult] = []
    for name, check in checks:
        try:
            check()
        except Exception as exc:  # noqa: BLE001 - CLI should report all failures.
            results.append(CheckResult(name, False, str(exc)))
        else:
            results.append(CheckResult(name, True, "ok"))
    return results


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[2],
        help="CodeWisdom-AutoRacer repository root",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    results = run_checks(args.repo_root.resolve())
    for result in results:
        status = "PASS" if result.passed else "FAIL"
        print(f"{status} {result.name}: {result.detail}")
    return 0 if all(result.passed for result in results) else 1


if __name__ == "__main__":
    sys.exit(main())
