#!/usr/bin/env python3
"""Offline fake-odom acceptance check for the phase-2 Pure Pursuit tracker."""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path


REQUIRED_CASES = [
    "straight_2m",
    "left_arc_r2m",
    "right_arc_r2m",
    "s_curve",
    "stop_at_end",
    "goal_2m_2m",
    "goal_3m_2m",
]


@dataclass
class CheckResult:
    name: str
    passed: bool
    detail: str


class TrackerFailure(AssertionError):
    pass


def import_tracker(repo_root: Path):
    package_dir = repo_root / "src" / "autoracer_path_tracking"
    if str(package_dir) not in sys.path:
        sys.path.insert(0, str(package_dir))
    from autoracer_path_tracking.pure_pursuit import (  # noqa: PLC0415
        ControllerConfig,
        PurePursuitController,
        Pose2D,
        load_fixture,
        poses_from_fixture,
    )

    return ControllerConfig, PurePursuitController, Pose2D, load_fixture, poses_from_fixture


def require(condition: bool, detail: str) -> None:
    if not condition:
        raise TrackerFailure(detail)


def has_positive_then_negative(values: list[float]) -> bool:
    seen_positive = False
    for value in values:
        if value > 0.02:
            seen_positive = True
        if seen_positive and value < -0.02:
            return True
    return False


def validate_case(case: str, fixture_dir: Path, repo_root: Path) -> None:
    ControllerConfig, PurePursuitController, Pose2D, load_fixture, poses_from_fixture = import_tracker(repo_root)
    fixture = load_fixture(fixture_dir / f"{case}.json")
    poses = poses_from_fixture(fixture)
    expected = fixture.get("expected", {})
    config = ControllerConfig(
        target_speed_mps=float(fixture.get("target_speed_mps", 0.2)),
        lookahead_m=0.60,
        goal_tolerance_m=float(expected.get("stop_distance_m", 0.05)),
        wheelbase_m=0.60,
        max_steering_angle_rad=0.262,
        max_target_speed_mps=0.25,
        allow_reverse=False,
    )
    controller = PurePursuitController(config)
    controller.set_path(poses, case)
    outputs = [controller.compute(pose, 0.0) for pose in poses]
    moving_outputs = [output for output in outputs if not output.stop_commanded]
    steering_values = [output.steering_angle_rad for output in moving_outputs]

    require(outputs, "tracker produced no outputs")
    require(all(abs(output.steering_angle_rad) <= config.max_steering_angle_rad + 1e-6 for output in outputs),
            "steering exceeded max_steering_angle_rad")
    require(all(output.speed_mps >= -1e-6 for output in outputs), "tracker emitted reverse speed")
    require(all(output.speed_mps <= float(fixture["target_speed_mps"]) + 0.02 for output in outputs),
            "tracker exceeded target speed tolerance")

    if case in ("straight_2m", "stop_at_end"):
        max_abs = float(expected.get("max_abs_steering_rad", 0.1))
        require(max((abs(value) for value in steering_values), default=0.0) <= max_abs,
                f"straight/stop steering exceeded {max_abs:.3f} rad")
        if case == "straight_2m":
            before_stop = controller.compute(Pose2D(1.90, 0.0, 0.0), 0.0)
            inside_stop = controller.compute(Pose2D(1.96, 0.0, 0.0), 0.0)
            require(not before_stop.stop_commanded, "straight_2m controller must not stop at the 0.10 m acceptance boundary")
            require(before_stop.remaining_distance_m > float(expected["stop_distance_m"]),
                    "straight_2m remaining distance must use continuous path progress")
            require(inside_stop.stop_commanded, "straight_2m controller must stop inside the 0.05 m terminal window")
    elif case == "left_arc_r2m":
        require(max(steering_values, default=0.0) > 0.02, "left arc did not command positive steering")
    elif case == "right_arc_r2m":
        require(min(steering_values, default=0.0) < -0.02, "right arc did not command negative steering")
    elif case == "s_curve":
        require(has_positive_then_negative(steering_values), "s_curve did not command positive then negative steering")
    elif case in ("goal_2m_2m", "goal_3m_2m"):
        require(max(steering_values, default=0.0) > 0.02, f"{case} did not initially command positive steering")
        expected = fixture.get("expected", {})
        require(poses[-1].x == float(expected["goal_x_m"]) and poses[-1].y == float(expected["goal_y_m"]),
                f"{case} final pose must match expected goal")
        dynamic_backtrack = PurePursuitController(config)
        dynamic_backtrack.set_path([
            Pose2D(2.835, 8.217, math.radians(88.3)),
            Pose2D(2.0, 2.0, math.radians(-97.6)),
        ], "goal_2m_2m_backtrack")
        backtrack_output = dynamic_backtrack.compute(Pose2D(2.835, 8.217, math.radians(88.3)), 0.0)
        require(backtrack_output.stop_commanded, "tracker must stop when a non-reversible target is behind the car")
        require(abs(backtrack_output.speed_mps) <= 1e-6, "non-reversible backtrack stop speed was not zero")
        require(backtrack_output.brake, "non-reversible backtrack stop did not set brake")

    final_output = outputs[-1]
    stop_distance = float(expected.get("stop_distance_m", 0.05))
    require(final_output.remaining_distance_m <= stop_distance + 1e-6, "final fake odom was not within stop distance")
    require(final_output.stop_commanded, "final output did not command stop")
    require(abs(final_output.speed_mps) <= 1e-6, "final stop speed was not zero")
    require(final_output.brake, "final stop did not set brake")


def run_checks(fixture_dir: Path, repo_root: Path) -> list[CheckResult]:
    results: list[CheckResult] = []
    for case in REQUIRED_CASES:
        try:
            validate_case(case, fixture_dir, repo_root)
        except Exception as exc:  # noqa: BLE001 - CLI should report all failures.
            results.append(CheckResult(case, False, str(exc)))
        else:
            results.append(CheckResult(case, True, "ok"))
    return results


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[2],
        help="CodeWisdom-AutoRacer repository root",
    )
    parser.add_argument(
        "--fixture-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "fixtures" / "stage2_paths",
        help="directory containing phase-2 path fixtures",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    results = run_checks(args.fixture_dir.resolve(), args.repo_root.resolve())
    for result in results:
        status = "PASS" if result.passed else "FAIL"
        print(f"{status} {result.name}: {result.detail}")
    return 0 if all(result.passed for result in results) else 1


if __name__ == "__main__":
    sys.exit(main())
