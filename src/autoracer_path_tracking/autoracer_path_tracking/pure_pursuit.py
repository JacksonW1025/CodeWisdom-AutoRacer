"""Pure Pursuit controller shared by the ROS node and offline acceptance."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ControllerConfig:
    wheelbase_m: float = 0.60
    max_steering_angle_rad: float = 0.262
    lookahead_m: float = 0.60
    goal_tolerance_m: float = 0.20
    target_speed_mps: float = 0.20
    max_target_speed_mps: float = 0.25
    allow_reverse: bool = False


@dataclass(frozen=True)
class TrackerOutput:
    speed_mps: float
    steering_angle_rad: float
    brake: bool
    target_index: int
    lookahead_m: float
    lateral_error_m: float
    heading_error_rad: float
    target_speed_mps: float
    steering_limited: bool
    speed_limited: bool
    stop_commanded: bool
    remaining_distance_m: float


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def load_fixture(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def poses_from_fixture(fixture: dict[str, Any]) -> list[Pose2D]:
    return [
        Pose2D(float(pose['x']), float(pose['y']), float(pose['yaw_rad']))
        for pose in fixture['poses']
    ]


def path_distances(poses: list[Pose2D]) -> list[float]:
    distances = [0.0]
    for previous, current in zip(poses, poses[1:]):
        distances.append(distances[-1] + math.hypot(current.x - previous.x, current.y - previous.y))
    return distances


class PurePursuitController:
    def __init__(self, config: ControllerConfig | None = None) -> None:
        self.config = config or ControllerConfig()
        self.poses: list[Pose2D] = []
        self.distances: list[float] = []
        self.case_name = ''

    def set_path(self, poses: list[Pose2D], case_name: str = '') -> None:
        if len(poses) < 2:
            raise ValueError('Pure Pursuit requires at least two path poses')
        self.poses = poses
        self.distances = path_distances(poses)
        self.case_name = case_name

    def compute(self, current: Pose2D, actual_speed_mps: float = 0.0) -> TrackerOutput:
        if not self.poses:
            return self._stop_output(current, 0, 0.0, 0.0, 0.0)

        nearest_index = self._nearest_index(current)
        remaining = max(0.0, self.distances[-1] - self.distances[nearest_index])
        lateral_error = self._signed_lateral_error(current, nearest_index)
        heading_error = normalize_angle(self.poses[nearest_index].yaw - current.yaw)

        if remaining <= self.config.goal_tolerance_m:
            return self._stop_output(current, nearest_index, remaining, lateral_error, heading_error)

        target_distance = min(self.distances[-1], self.distances[nearest_index] + self.config.lookahead_m)
        target_index, target_pose = self._interpolate_at_distance(target_distance)
        local_x, local_y = self._to_local(current, target_pose)
        lookahead = max(0.001, math.hypot(local_x, local_y))

        if local_x < 0.0 and not self.config.allow_reverse:
            steering = 0.0
        else:
            curvature = 2.0 * local_y / (lookahead * lookahead)
            steering = math.atan(self.config.wheelbase_m * curvature)

        limited_steering = max(
            -self.config.max_steering_angle_rad,
            min(self.config.max_steering_angle_rad, steering),
        )
        target_speed = min(self.config.target_speed_mps, self.config.max_target_speed_mps)
        if not self.config.allow_reverse:
            target_speed = max(0.0, target_speed)

        return TrackerOutput(
            speed_mps=target_speed,
            steering_angle_rad=limited_steering,
            brake=False,
            target_index=target_index,
            lookahead_m=lookahead,
            lateral_error_m=lateral_error,
            heading_error_rad=heading_error,
            target_speed_mps=target_speed,
            steering_limited=abs(limited_steering - steering) > 1e-6,
            speed_limited=(
                self.config.target_speed_mps > self.config.max_target_speed_mps
                or (not self.config.allow_reverse and self.config.target_speed_mps < 0.0)
            ),
            stop_commanded=False,
            remaining_distance_m=remaining,
        )

    def _stop_output(
        self,
        current: Pose2D,
        target_index: int,
        remaining: float,
        lateral_error: float,
        heading_error: float,
    ) -> TrackerOutput:
        return TrackerOutput(
            speed_mps=0.0,
            steering_angle_rad=0.0,
            brake=True,
            target_index=target_index,
            lookahead_m=self.config.lookahead_m,
            lateral_error_m=lateral_error,
            heading_error_rad=heading_error,
            target_speed_mps=0.0,
            steering_limited=False,
            speed_limited=False,
            stop_commanded=True,
            remaining_distance_m=remaining,
        )

    def _nearest_index(self, current: Pose2D) -> int:
        return min(
            range(len(self.poses)),
            key=lambda index: math.hypot(self.poses[index].x - current.x, self.poses[index].y - current.y),
        )

    def _interpolate_at_distance(self, distance_m: float) -> tuple[int, Pose2D]:
        if distance_m <= 0.0:
            return 0, self.poses[0]
        if distance_m >= self.distances[-1]:
            return len(self.poses) - 1, self.poses[-1]

        for index in range(1, len(self.distances)):
            if self.distances[index] < distance_m:
                continue
            previous = self.poses[index - 1]
            current = self.poses[index]
            span = self.distances[index] - self.distances[index - 1]
            ratio = 0.0 if span <= 1e-9 else (distance_m - self.distances[index - 1]) / span
            yaw_delta = normalize_angle(current.yaw - previous.yaw)
            return index, Pose2D(
                previous.x + (current.x - previous.x) * ratio,
                previous.y + (current.y - previous.y) * ratio,
                normalize_angle(previous.yaw + yaw_delta * ratio),
            )
        return len(self.poses) - 1, self.poses[-1]

    @staticmethod
    def _to_local(current: Pose2D, target: Pose2D) -> tuple[float, float]:
        dx = target.x - current.x
        dy = target.y - current.y
        cos_yaw = math.cos(current.yaw)
        sin_yaw = math.sin(current.yaw)
        return cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy

    def _signed_lateral_error(self, current: Pose2D, nearest_index: int) -> float:
        if len(self.poses) < 2:
            return 0.0
        if nearest_index >= len(self.poses) - 1:
            start = self.poses[nearest_index - 1]
            end = self.poses[nearest_index]
        else:
            start = self.poses[nearest_index]
            end = self.poses[nearest_index + 1]

        segment_x = end.x - start.x
        segment_y = end.y - start.y
        segment_length = math.hypot(segment_x, segment_y)
        if segment_length <= 1e-9:
            return 0.0
        return ((current.x - start.x) * segment_y - (current.y - start.y) * segment_x) / segment_length
