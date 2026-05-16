#!/usr/bin/env python3
"""Validate phase-2 path fixtures before tracker implementation."""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


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


class FixtureFailure(AssertionError):
    pass


def require(condition: bool, detail: str) -> None:
    if not condition:
        raise FixtureFailure(detail)


def require_number(value: Any, label: str) -> float:
    require(isinstance(value, (int, float)), f"{label} must be numeric")
    number = float(value)
    require(math.isfinite(number), f"{label} must be finite")
    return number


def path_length(poses: list[dict[str, Any]]) -> float:
    total = 0.0
    for previous, current in zip(poses, poses[1:]):
        total += math.hypot(float(current["x"]) - float(previous["x"]), float(current["y"]) - float(previous["y"]))
    return total


def signed_area(poses: list[dict[str, Any]]) -> float:
    area = 0.0
    for previous, current in zip(poses, poses[1:]):
        area += float(previous["x"]) * float(current["y"]) - float(current["x"]) * float(previous["y"])
    return area * 0.5


def validate_terminal_distances(expected: dict[str, Any]) -> None:
    stop_distance = require_number(expected.get("stop_distance_m", 0.05), "expected.stop_distance_m")
    acceptance_distance = require_number(expected.get("acceptance_distance_m", 0.10), "expected.acceptance_distance_m")
    require(stop_distance <= 0.05 + 1e-6, "controller stop distance must be <= 0.05 m")
    require(acceptance_distance >= stop_distance, "acceptance distance must not be tighter than control stop distance")
    require(acceptance_distance <= 0.10 + 1e-6, "real-car acceptance distance must be <= 0.10 m")


def load_fixture(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise FixtureFailure(f"{path.name} is not valid JSON: {exc}") from exc


def validate_common(case: str, fixture: dict[str, Any]) -> list[dict[str, Any]]:
    require(fixture.get("case") == case, f"case field must be {case}")
    require(fixture.get("fixture_version") == 1, "fixture_version must be 1")
    require(fixture.get("message_semantics") == "nav_msgs/Path", "message_semantics must be nav_msgs/Path")
    require(fixture.get("frame_id") == "odom", "frame_id must be odom")
    target_speed = require_number(fixture.get("target_speed_mps"), "target_speed_mps")
    require(0.0 < target_speed <= 0.25, "target_speed_mps must be low speed <= 0.25")
    require(fixture.get("allow_reverse") is False, "allow_reverse must be false for phase-2 fixtures")

    poses = fixture.get("poses")
    require(isinstance(poses, list) and len(poses) >= 2, "poses must contain at least two entries")
    previous: dict[str, Any] | None = None
    for index, pose in enumerate(poses):
        require(isinstance(pose, dict), f"pose {index} must be an object")
        x = require_number(pose.get("x"), f"pose {index}.x")
        y = require_number(pose.get("y"), f"pose {index}.y")
        yaw = require_number(pose.get("yaw_rad"), f"pose {index}.yaw_rad")
        require(-math.pi <= yaw <= math.pi, f"pose {index}.yaw_rad must be normalized")
        if previous is not None:
            distance = math.hypot(x - float(previous["x"]), y - float(previous["y"]))
            require(distance > 0.02, f"pose {index} is too close to previous pose")
        previous = pose
    return poses


def validate_straight(fixture: dict[str, Any], poses: list[dict[str, Any]]) -> None:
    expected = fixture.get("expected", {})
    require(isinstance(expected, dict), "straight fixture must include expected object")
    validate_terminal_distances(expected)
    require(math.isclose(path_length(poses), 2.0, abs_tol=0.05), "straight_2m length must be about 2 m")
    require(max(abs(float(pose["y"])) for pose in poses) <= 1e-6, "straight_2m y must stay zero")
    require(max(abs(float(pose["yaw_rad"])) for pose in poses) <= 1e-6, "straight_2m yaw must stay zero")


def validate_left_arc(poses: list[dict[str, Any]]) -> None:
    require(path_length(poses) > 1.8, "left arc length too short")
    require(float(poses[-1]["yaw_rad"]) > 0.9, "left arc final yaw must be positive")
    require(signed_area(poses) > 0.05, "left arc must curve left")


def validate_right_arc(poses: list[dict[str, Any]]) -> None:
    require(path_length(poses) > 1.8, "right arc length too short")
    require(float(poses[-1]["yaw_rad"]) < -0.9, "right arc final yaw must be negative")
    require(signed_area(poses) < -0.05, "right arc must curve right")


def validate_s_curve(poses: list[dict[str, Any]]) -> None:
    y_values = [float(pose["y"]) for pose in poses]
    yaw_values = [float(pose["yaw_rad"]) for pose in poses]
    require(max(y_values) > 0.15 and min(y_values) < -0.15, "s_curve must include left and right lateral offsets")
    require(max(yaw_values) > 0.10 and min(yaw_values) < -0.10, "s_curve must include yaw sign changes")
    require(path_length(poses) >= 3.5, "s_curve path length too short")


def validate_stop_at_end(fixture: dict[str, Any], poses: list[dict[str, Any]]) -> None:
    require(fixture.get("stop_at_end") is True, "stop_at_end fixture must require terminal stop")
    terminal_speed = require_number(fixture.get("terminal_speed_mps"), "terminal_speed_mps")
    require(math.isclose(terminal_speed, 0.0, abs_tol=1e-6), "terminal_speed_mps must be 0")
    expected = fixture.get("expected", {})
    require(isinstance(expected, dict), "stop_at_end fixture must include expected object")
    validate_terminal_distances(expected)
    require(path_length(poses) >= 1.4, "stop_at_end path length too short")


def validate_goal_fixture(fixture: dict[str, Any], poses: list[dict[str, Any]]) -> None:
    require(fixture.get("stop_at_end") is True, "goal fixture must require terminal stop")
    expected = fixture.get("expected", {})
    require(isinstance(expected, dict), "goal fixture must include expected object")
    goal_x = require_number(expected.get("goal_x_m"), "expected.goal_x_m")
    goal_y = require_number(expected.get("goal_y_m"), "expected.goal_y_m")
    validate_terminal_distances(expected)
    require(math.isclose(float(poses[0]["x"]), 0.0, abs_tol=1e-6), "goal path must start at x=0")
    require(math.isclose(float(poses[0]["y"]), 0.0, abs_tol=1e-6), "goal path must start at y=0")
    require(math.isclose(float(poses[-1]["x"]), goal_x, abs_tol=0.02), "goal path final x mismatch")
    require(math.isclose(float(poses[-1]["y"]), goal_y, abs_tol=0.02), "goal path final y mismatch")
    expected_length = math.hypot(goal_x, goal_y)
    require(math.isclose(path_length(poses), expected_length, abs_tol=0.05), "goal path length mismatch")
    require(max(float(pose["x"]) for pose in poses) <= goal_x + 0.02, "goal path x overshoots target")
    require(max(float(pose["y"]) for pose in poses) <= goal_y + 0.02, "goal path y overshoots target")
    require(all(float(pose["x"]) >= -1e-6 and float(pose["y"]) >= -1e-6 for pose in poses),
            "goal path must remain in positive quadrant")


def validate_case(case: str, fixture_dir: Path) -> None:
    path = fixture_dir / f"{case}.json"
    require(path.exists(), f"missing fixture {path}")
    fixture = load_fixture(path)
    poses = validate_common(case, fixture)

    if case == "straight_2m":
        validate_straight(fixture, poses)
    elif case == "left_arc_r2m":
        validate_left_arc(poses)
    elif case == "right_arc_r2m":
        validate_right_arc(poses)
    elif case == "s_curve":
        validate_s_curve(poses)
    elif case == "stop_at_end":
        validate_stop_at_end(fixture, poses)
    elif case in ("goal_2m_2m", "goal_3m_2m"):
        validate_goal_fixture(fixture, poses)


def run_checks(fixture_dir: Path) -> list[CheckResult]:
    results: list[CheckResult] = []
    for case in REQUIRED_CASES:
        try:
            validate_case(case, fixture_dir)
        except Exception as exc:  # noqa: BLE001 - command-line report should include all fixture failures.
            results.append(CheckResult(case, False, str(exc)))
        else:
            results.append(CheckResult(case, True, "ok"))
    return results


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--fixture-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "fixtures" / "stage2_paths",
        help="directory containing phase-2 path fixtures",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    results = run_checks(args.fixture_dir.resolve())
    for result in results:
        status = "PASS" if result.passed else "FAIL"
        print(f"{status} {result.name}: {result.detail}")
    return 0 if all(result.passed for result in results) else 1


if __name__ == "__main__":
    sys.exit(main())
