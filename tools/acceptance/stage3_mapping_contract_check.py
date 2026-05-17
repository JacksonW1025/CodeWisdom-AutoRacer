#!/usr/bin/env python3
"""Offline contract checks for phase-3 mapping launch and parameters."""

from __future__ import annotations

import argparse
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


def check_stage3_launch(repo_root: Path) -> None:
    launch = repo_root / "src" / "autoracer_bringup" / "launch" / "stage3_mapping.launch.py"
    text = launch.read_text(encoding="utf-8")

    require_text(text, "stage1_chassis.launch.py", "stage3 launch must include phase-1 chassis entry")
    require_text(text, "imu_spec_msg.launch.py", "stage3 launch must include IMU raw driver")
    require_text(text, "imu_filter_madgwick_node", "stage3 launch must include Madgwick /imu/data filter")
    require_text(text, "lslidar_cx_launch.py", "stage3 launch must include LiDAR launch")
    require_text(text, "slam.launch.py", "stage3 launch must include slam_toolbox launch")
    require_text(text, "robot_description.launch.py", "stage3 launch must include URDF/TF launch")
    require_text(text, "start_robot_description", "stage3 launch must allow URDF/TF dry-launch control")
    require_text(text, "'include_bringup': 'false'", "stage3 must force slam include_bringup=false")
    require_text(text, "counts_per_meter", "stage3 launch must expose counts_per_meter")
    require_text(text, "validate_stage3_required_inputs", "stage3 launch must validate required live inputs")
    require_text(text, "counts_per_meter must be > 0", "stage3 launch must reject uncalibrated counts_per_meter")
    require_text(text, "start_imu", "stage3 launch must allow IMU dry-launch control")
    require_text(text, "start_chassis", "stage3 launch must allow non-hardware dry launch by disabling chassis")
    require_text(text, "start_lidar", "stage3 launch must allow non-hardware dry launch by disabling LiDAR")
    require_text(text, "odom_frame", "stage3 launch must expose odom_frame")
    require_text(text, "slam_params_file", "stage3 launch must expose selectable slam_toolbox params")
    require_text(text, "LaunchConfiguration('slam_params_file')", "stage3 launch must pass slam params into slam launch")
    require("/cmd_vel" not in text, "stage3 launch must not introduce legacy /cmd_vel")


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


def check_slam_launch(repo_root: Path) -> None:
    launch = repo_root / "src" / "autoracer_robot_slam" / "autoracer_slam_toolbox" / "launch" / "slam.launch.py"
    text = launch.read_text(encoding="utf-8")

    required_pairs = {
        "'target_frame': 'base_link'": "target_frame must be base_link",
        "'transform_tolerance': 0.01": "transform_tolerance must be 0.01",
        "'min_height': 0.1": "min_height must be 0.1",
        "'max_height': 1.5": "max_height must be 1.5",
        "'angle_min': -3.14159": "angle_min must be -pi",
        "'angle_max': 3.14159": "angle_max must be +pi",
        "'angle_increment': 0.00436": "angle_increment must be about 0.25 deg",
        "'scan_time': 0.05": "scan_time must be 0.05",
        "'range_min': 0.2": "range_min must be 0.2",
        "'range_max': 20.0": "range_max must be 20.0",
        "'use_inf': True": "use_inf must be true",
        "('cloud_in', '/point_cloud_raw')": "cloud input must be /point_cloud_raw",
        "('scan', '/scan')": "scan output must be /scan",
        "slam_params_file": "slam launch must expose selectable slam params",
        "mapper_params_online_sync.yaml": "default slam params must remain the online sync baseline",
    }
    for needle, detail in required_pairs.items():
        require_text(text, needle, detail)


def check_slam_params(repo_root: Path) -> None:
    params = repo_root / "src" / "autoracer_robot_slam" / "autoracer_slam_toolbox" / "config" / "mapper_params_online_sync.yaml"
    text = params.read_text(encoding="utf-8")

    required = {
        "odom_frame: odom": "slam_toolbox must use canonical odom by default",
        "map_frame: map": "slam_toolbox must publish map frame",
        "base_frame: base_footprint": "slam_toolbox must use base_footprint",
        "scan_topic: /scan": "slam_toolbox must consume /scan",
        "mode: mapping": "slam_toolbox must run in mapping mode",
        "map_update_interval: 5.0": "map update interval must be pinned",
        "resolution: 0.05": "map resolution must be pinned",
        "max_laser_range: 20.0": "max laser range must match stage contract",
        "minimum_time_interval: 0.5": "minimum processing interval must be pinned",
        "transform_timeout: 0.2": "transform timeout must be pinned",
        "minimum_travel_distance: 0.5": "scan trigger distance must be pinned",
        "minimum_travel_heading: 0.5": "scan trigger heading must be pinned",
        "do_loop_closing: true": "loop closing must be enabled unless explicitly changed",
    }
    for needle, detail in required.items():
        require_text(text, needle, detail)


def check_dense_slam_params(repo_root: Path) -> None:
    params = repo_root / "src" / "autoracer_robot_slam" / "autoracer_slam_toolbox" / "config" / "mapper_params_floor2_dense.yaml"
    text = params.read_text(encoding="utf-8")

    required = {
        "odom_frame: odom": "dense slam profile must use canonical odom",
        "scan_topic: /scan": "dense slam profile must consume 2D scan, not raw cloud",
        "resolution: 0.05": "dense slam profile must keep 5 cm map resolution",
        "throttle_scans: 2": "dense slam profile must reduce live laptop processing load",
        "minimum_time_interval: 0.20": "dense slam profile must process scans more frequently than the baseline without overrunning",
        "minimum_travel_distance: 0.18": "dense slam profile must insert scans below the baseline distance interval",
        "minimum_travel_heading: 0.12": "dense slam profile must insert scans during modest turns",
        "scan_buffer_size: 20": "dense slam profile must keep a bounded local scan buffer",
        "do_loop_closing: true": "dense slam profile must keep loop closure enabled",
        "This is not a 3D point-cloud matching profile": "dense profile must document 2D-vs-3D boundary",
    }
    for needle, detail in required.items():
        require_text(text, needle, detail)


def check_map_save_launch(repo_root: Path) -> None:
    launch = repo_root / "src" / "autoracer_robot_nav2" / "launch" / "save_map.launch.py"
    text = launch.read_text(encoding="utf-8")

    required = {
        "output_dir": "map saver must expose an explicit output_dir",
        "artifacts/maps": "map saver default must write to an artifacts directory, not package share",
        "Path.cwd() / output_dir": "relative output_dir must resolve from the operator's working directory",
        "output_dir.mkdir": "map saver must create the output directory before saving",
        "map_saver_cli": "map saver must use nav2_map_server map_saver_cli",
    }
    for needle, detail in required.items():
        require_text(text, needle, detail)
    require("get_package_share_directory" not in text, "map saver must not default to package share output")
    require("src_map_dir" not in text, "map saver must not pretend to write a separate src backup under symlink-install")


def run_checks(repo_root: Path) -> list[CheckResult]:
    checks = [
        ("stage3_launch", lambda: check_stage3_launch(repo_root)),
        ("robot_description_tf", lambda: check_robot_description_tf(repo_root)),
        ("slam_launch", lambda: check_slam_launch(repo_root)),
        ("slam_params", lambda: check_slam_params(repo_root)),
        ("dense_slam_params", lambda: check_dense_slam_params(repo_root)),
        ("map_save_launch", lambda: check_map_save_launch(repo_root)),
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
