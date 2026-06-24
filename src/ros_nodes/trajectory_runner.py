#!/usr/bin/env python3
"""Publish velocity commands for configurable Warthog test trajectories."""

from __future__ import annotations

import abc
import bisect
import math
import threading
from dataclasses import dataclass
from typing import Callable, Dict, List, Sequence, Tuple

import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float
    angular_z: float

    def to_twist(self) -> Twist:
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        return msg


class PoseStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pose = Pose2D(0.0, 0.0, 0.0)
        self._ready = False

    def update(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            (ori.x, ori.y, ori.z, ori.w)
        )
        with self._lock:
            self._pose = Pose2D(pos.x, pos.y, yaw)
            self._ready = True

    def get(self) -> Tuple[Pose2D, bool]:
        with self._lock:
            return self._pose, self._ready


class Trajectory(abc.ABC):
    name = "base"

    @abc.abstractmethod
    def command(self, elapsed: float, pose: Pose2D) -> VelocityCommand:
        pass


class AnalyticalTrajectory(Trajectory):
    def __init__(
        self,
        name: str,
        command_fn: Callable[[float, Pose2D], VelocityCommand],
    ) -> None:
        self.name = name
        self._command_fn = command_fn

    def command(self, elapsed: float, pose: Pose2D) -> VelocityCommand:
        return self._command_fn(elapsed, pose)


class SegmentTrajectory(Trajectory):
    """Piecewise-constant velocity commands: [(vx, wz, duration_sec), ...]."""

    name = "segments"

    def __init__(
        self,
        segments: Sequence[Tuple[float, float, float]],
        repeat: bool = True,
    ) -> None:
        if not segments:
            raise ValueError("SegmentTrajectory requires at least one segment")
        self._segments = list(segments)
        self._repeat = repeat
        self._ends = []
        total = 0.0
        for _, _, duration in self._segments:
            if duration <= 0.0:
                raise ValueError("Segment durations must be positive")
            total += duration
            self._ends.append(total)
        self._total_duration = total

    def command(self, elapsed: float, pose: Pose2D) -> VelocityCommand:
        if self._repeat:
            elapsed = elapsed % self._total_duration
        index = bisect.bisect_right(self._ends, elapsed)
        index = min(index, len(self._segments) - 1)
        vx, wz, _ = self._segments[index]
        return VelocityCommand(vx, wz)


class InterpolatedVelocityTrajectory(Trajectory):
    """Linearly interpolated velocity keyframes: [(time_sec, vx, wz), ...]."""

    name = "interpolated"

    def __init__(
        self,
        keyframes: Sequence[Tuple[float, float, float]],
        repeat: bool = True,
    ) -> None:
        if len(keyframes) < 2:
            raise ValueError("Interpolated trajectory requires at least two keyframes")
        ordered = sorted(keyframes, key=lambda item: item[0])
        if ordered[0][0] != 0.0:
            raise ValueError("First keyframe must start at t=0.0")
        self._times = [frame[0] for frame in ordered]
        self._vx = [frame[1] for frame in ordered]
        self._wz = [frame[2] for frame in ordered]
        self._repeat = repeat
        self._total_duration = self._times[-1]
        if self._total_duration <= 0.0:
            raise ValueError("Last keyframe time must be positive")

    def command(self, elapsed: float, pose: Pose2D) -> VelocityCommand:
        if self._repeat:
            elapsed = elapsed % self._total_duration
        if elapsed <= self._times[0]:
            return VelocityCommand(self._vx[0], self._wz[0])
        if elapsed >= self._times[-1]:
            return VelocityCommand(self._vx[-1], self._wz[-1])

        right = bisect.bisect_right(self._times, elapsed)
        left = right - 1
        span = self._times[right] - self._times[left]
        alpha = (elapsed - self._times[left]) / span
        vx = lerp(self._vx[left], self._vx[right], alpha)
        wz = lerp(self._wz[left], self._wz[right], alpha)
        return VelocityCommand(vx, wz)


class WaypointTrajectory(Trajectory):
    """Simple pose-feedback controller for a cyclic list of waypoints."""

    name = "waypoints"

    def __init__(
        self,
        waypoints: Sequence[Tuple[float, float]],
        target_speed: float,
        yaw_gain: float,
        reach_radius: float,
        max_angular_speed: float,
    ) -> None:
        if not waypoints:
            raise ValueError("WaypointTrajectory requires at least one waypoint")
        self._waypoints = list(waypoints)
        self._target_speed = target_speed
        self._yaw_gain = yaw_gain
        self._reach_radius = reach_radius
        self._max_angular_speed = max_angular_speed
        self._index = 0

    def command(self, elapsed: float, pose: Pose2D) -> VelocityCommand:
        target = self._waypoints[self._index]
        dx = target[0] - pose.x
        dy = target[1] - pose.y
        distance = math.hypot(dx, dy)

        if distance <= self._reach_radius:
            self._index = (self._index + 1) % len(self._waypoints)
            target = self._waypoints[self._index]
            dx = target[0] - pose.x
            dy = target[1] - pose.y
            distance = math.hypot(dx, dy)

        heading_error = normalize_angle(math.atan2(dy, dx) - pose.yaw)
        angular = clamp(
            self._yaw_gain * heading_error,
            -self._max_angular_speed,
            self._max_angular_speed,
        )
        linear = self._target_speed * max(0.0, math.cos(heading_error))
        if distance < self._reach_radius * 2.0:
            linear *= max(0.2, distance / (self._reach_radius * 2.0))
        return VelocityCommand(linear, angular)


class TrajectoryFactory:
    def __init__(self) -> None:
        self._builders: Dict[str, Callable[[], Trajectory]] = {}

    def register(self, name: str, builder: Callable[[], Trajectory]) -> None:
        self._builders[name] = builder

    def create(self, name: str) -> Trajectory:
        if name not in self._builders:
            available = ", ".join(sorted(self._builders))
            raise ValueError(f"Unknown trajectory '{name}'. Available: {available}")
        return self._builders[name]()


class TrajectoryRunner:
    def __init__(self) -> None:
        self._pose_store = PoseStore()
        self._params = self._load_params()
        self._factory = self._build_factory()
        self._trajectory = self._factory.create(self._params.trajectory_type)
        self._publisher = rospy.Publisher(
            self._params.cmd_vel_topic,
            Twist,
            queue_size=10,
        )
        rospy.Subscriber(
            self._params.odom_topic,
            Odometry,
            self._pose_store.update,
        )

    def spin(self) -> None:
        self._wait_for_odom()

        rate = rospy.Rate(self._params.publish_rate)
        start = rospy.Time.now()
        rospy.loginfo(
            "Starting trajectory '%s' for %.1f seconds, cmd_vel=%s, odom=%s",
            self._trajectory.name,
            self._params.duration_sec,
            self._params.cmd_vel_topic,
            self._params.odom_topic,
        )

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed >= self._params.duration_sec:
                break

            pose, _ = self._pose_store.get()
            command = self._trajectory.command(elapsed, pose)
            command = self._limit(command)
            self._publisher.publish(command.to_twist())

            rospy.loginfo_throttle(
                10.0,
                "trajectory=%s t=%.1f pose=(%.2f, %.2f, %.2f) cmd=(%.2f, %.2f)",
                self._trajectory.name,
                elapsed,
                pose.x,
                pose.y,
                pose.yaw,
                command.linear_x,
                command.angular_z,
            )
            rate.sleep()

        self.stop()
        rospy.loginfo("Trajectory complete")

    def stop(self) -> None:
        self._publisher.publish(Twist())

    def _wait_for_odom(self) -> None:
        rospy.loginfo("Waiting for odometry on %s...", self._params.odom_topic)
        while not rospy.is_shutdown():
            _, ready = self._pose_store.get()
            if ready:
                return
            rospy.sleep(0.1)

    def _limit(self, command: VelocityCommand) -> VelocityCommand:
        return VelocityCommand(
            clamp(command.linear_x, -self._params.max_linear_speed, self._params.max_linear_speed),
            clamp(command.angular_z, -self._params.max_angular_speed, self._params.max_angular_speed),
        )

    def _build_factory(self) -> TrajectoryFactory:
        factory = TrajectoryFactory()
        factory.register("lemniscate", self._build_lemniscate)
        factory.register("circle", self._build_circle)
        factory.register("segments", self._build_segments)
        factory.register("interpolated", self._build_interpolated)
        factory.register("waypoints", self._build_waypoints)
        return factory

    def _build_lemniscate(self) -> Trajectory:
        p = self._params

        def command(elapsed: float, pose: Pose2D) -> VelocityCommand:
            omega = 2.0 * math.pi / p.lemniscate_period
            phase = omega * elapsed
            shape_v = p.lemniscate_scale * omega * math.cos(phase)
            shape_w = (
                p.lemniscate_scale
                * omega
                * math.sin(phase)
                * math.cos(phase)
                / (1.0 + math.sin(phase) ** 2)
            )
            norm = max(math.hypot(shape_v, shape_w), 1e-6)
            return VelocityCommand(
                p.target_speed * shape_v / norm,
                p.target_speed * shape_w / norm,
            )

        return AnalyticalTrajectory("lemniscate", command)

    def _build_circle(self) -> Trajectory:
        p = self._params

        def command(elapsed: float, pose: Pose2D) -> VelocityCommand:
            angular = p.target_speed / max(p.circle_radius, 1e-6)
            return VelocityCommand(p.target_speed, angular)

        return AnalyticalTrajectory("circle", command)

    def _build_segments(self) -> Trajectory:
        return SegmentTrajectory(
            self._params.segments,
            repeat=self._params.repeat_trajectory,
        )

    def _build_interpolated(self) -> Trajectory:
        return InterpolatedVelocityTrajectory(
            self._params.velocity_keyframes,
            repeat=self._params.repeat_trajectory,
        )

    def _build_waypoints(self) -> Trajectory:
        p = self._params
        return WaypointTrajectory(
            p.waypoints,
            target_speed=p.target_speed,
            yaw_gain=p.waypoint_yaw_gain,
            reach_radius=p.waypoint_reach_radius,
            max_angular_speed=p.max_angular_speed,
        )

    def _load_params(self) -> "RunnerParams":
        return RunnerParams(
            trajectory_type=rospy.get_param("~trajectory_type", "lemniscate"),
            duration_sec=float(rospy.get_param("~duration_sec", 30.0 * 60.0)),
            publish_rate=float(rospy.get_param("~publish_rate", 20.0)),
            odom_topic=rospy.get_param("~odom_topic", "/ground_truth/odom"),
            cmd_vel_topic=rospy.get_param("~cmd_vel_topic", "/cmd_vel"),
            target_speed=float(rospy.get_param("~target_speed", 0.5)),
            max_linear_speed=float(rospy.get_param("~max_linear_speed", 0.7)),
            max_angular_speed=float(rospy.get_param("~max_angular_speed", 1.5)),
            lemniscate_scale=float(rospy.get_param("~lemniscate_scale", 3.0)),
            lemniscate_period=float(rospy.get_param("~lemniscate_period", 40.0)),
            circle_radius=float(rospy.get_param("~circle_radius", 3.0)),
            repeat_trajectory=bool(rospy.get_param("~repeat_trajectory", True)),
            segments=parse_segments(rospy.get_param("~segments", default_segments())),
            velocity_keyframes=parse_keyframes(
                rospy.get_param("~velocity_keyframes", default_velocity_keyframes())
            ),
            waypoints=parse_waypoints(rospy.get_param("~waypoints", default_waypoints())),
            waypoint_yaw_gain=float(rospy.get_param("~waypoint_yaw_gain", 1.8)),
            waypoint_reach_radius=float(rospy.get_param("~waypoint_reach_radius", 0.5)),
        )


@dataclass(frozen=True)
class RunnerParams:
    trajectory_type: str
    duration_sec: float
    publish_rate: float
    odom_topic: str
    cmd_vel_topic: str
    target_speed: float
    max_linear_speed: float
    max_angular_speed: float
    lemniscate_scale: float
    lemniscate_period: float
    circle_radius: float
    repeat_trajectory: bool
    segments: List[Tuple[float, float, float]]
    velocity_keyframes: List[Tuple[float, float, float]]
    waypoints: List[Tuple[float, float]]
    waypoint_yaw_gain: float
    waypoint_reach_radius: float


def default_segments() -> List[List[float]]:
    return [
        [0.5, 0.0, 8.0],
        [0.0, 0.5, 3.15],
        [0.5, 0.0, 8.0],
        [0.0, 0.5, 3.15],
    ]


def default_velocity_keyframes() -> List[List[float]]:
    return [
        [0.0, 0.0, 0.0],
        [2.0, 0.5, 0.0],
        [12.0, 0.5, 0.4],
        [22.0, 0.5, -0.4],
        [32.0, 0.5, 0.0],
        [34.0, 0.0, 0.0],
    ]


def default_waypoints() -> List[List[float]]:
    return [
        [3.0, 0.0],
        [0.0, 3.0],
        [-3.0, 0.0],
        [0.0, -3.0],
    ]


def parse_segments(value: Sequence[Sequence[float]]) -> List[Tuple[float, float, float]]:
    return [tuple3(item, "segment") for item in value]


def parse_keyframes(value: Sequence[Sequence[float]]) -> List[Tuple[float, float, float]]:
    return [tuple3(item, "velocity keyframe") for item in value]


def parse_waypoints(value: Sequence[Sequence[float]]) -> List[Tuple[float, float]]:
    return [tuple2(item, "waypoint") for item in value]


def tuple3(item: Sequence[float], label: str) -> Tuple[float, float, float]:
    if len(item) != 3:
        raise ValueError(f"Each {label} must contain exactly three numbers")
    return float(item[0]), float(item[1]), float(item[2])


def tuple2(item: Sequence[float], label: str) -> Tuple[float, float]:
    if len(item) != 2:
        raise ValueError(f"Each {label} must contain exactly two numbers")
    return float(item[0]), float(item[1])


def lerp(left: float, right: float, alpha: float) -> float:
    return left + (right - left) * alpha


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def main() -> None:
    rospy.init_node("trajectory_runner")
    try:
        runner = TrajectoryRunner()
        runner.spin()
    except Exception as exc:
        rospy.logerr("Trajectory runner failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
