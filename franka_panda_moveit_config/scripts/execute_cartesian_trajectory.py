#!/usr/bin/env python3
"""Execute Cartesian pose waypoints with MoveIt.

Trajectory file example:

frame_id: fer_link0
group_name: panda_arm
link_name: fer_link8
max_step: 0.005
avoid_collisions: true
waypoints:
  # [x, y, z, qx, qy, qz, qw]
  - [0.40, 0.00, 0.35, 0.0, 1.0, 0.0, 0.0]
  - position: [0.42, 0.03, 0.35]
    orientation: [0.0, 1.0, 0.0, 0.0]
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node

try:
    import yaml
except ImportError:  # pragma: no cover - JSON still works without PyYAML.
    yaml = None


DEFAULT_FRAME_ID = "fer_link0"
DEFAULT_GROUP_NAME = "panda_arm"
DEFAULT_LINK_NAME = "fer_link8"
DEFAULT_MAX_STEP = 0.005
DEFAULT_MIN_FRACTION = 0.99
DEFAULT_MAX_JOINT_VELOCITY = 0.20
DEFAULT_MIN_SEGMENT_DURATION = 0.02


def load_trajectory_file(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        data = json.loads(text)
    else:
        if yaml is None:
            raise RuntimeError("PyYAML is not installed; use JSON or install python3-yaml.")
        data = yaml.safe_load(text)

    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError("Trajectory file must contain a mapping with a 'waypoints' entry.")
    return data


def sequence_as_floats(value: Any, expected_length: int, field_name: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != expected_length:
        raise ValueError(f"'{field_name}' must be a sequence of {expected_length} numbers.")
    return [float(item) for item in value]


def vector_from_value(value: Any, keys: tuple[str, ...], field_name: str) -> list[float]:
    if isinstance(value, dict):
        try:
            return [float(value[key]) for key in keys]
        except KeyError as exc:
            raise ValueError(f"'{field_name}' is missing key '{exc.args[0]}'.") from exc
    return sequence_as_floats(value, len(keys), field_name)


def normalize_quaternion(quaternion: list[float]) -> list[float]:
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm < 1e-9:
        raise ValueError("Quaternion norm is zero.")
    return [component / norm for component in quaternion]


def quaternion_from_value(value: Any) -> list[float]:
    if isinstance(value, dict) and all(key in value for key in ("qx", "qy", "qz", "qw")):
        return [float(value[key]) for key in ("qx", "qy", "qz", "qw")]
    if isinstance(value, dict) and all(key in value for key in ("x", "y", "z", "w")):
        return [float(value[key]) for key in ("x", "y", "z", "w")]
    return sequence_as_floats(value, 4, "orientation")


def pose_from_waypoint(waypoint: Any) -> Pose:
    if isinstance(waypoint, (list, tuple)):
        values = sequence_as_floats(waypoint, 7, "waypoint")
        position = values[:3]
        quaternion = values[3:]
    elif isinstance(waypoint, dict):
        if "position" in waypoint:
            position = vector_from_value(waypoint["position"], ("x", "y", "z"), "position")
        else:
            position = vector_from_value(waypoint, ("x", "y", "z"), "waypoint")

        orientation_value = waypoint.get("orientation", waypoint.get("quaternion", waypoint))
        quaternion = quaternion_from_value(orientation_value)
    else:
        raise ValueError("Each waypoint must be either a 7-element list or a mapping.")

    quaternion = normalize_quaternion(quaternion)

    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose


def duration_from_seconds(seconds: float) -> Duration:
    whole_seconds = math.floor(seconds)
    nanoseconds = round((seconds - whole_seconds) * 1_000_000_000)
    if nanoseconds == 1_000_000_000:
        whole_seconds += 1
        nanoseconds = 0

    duration = Duration()
    duration.sec = int(whole_seconds)
    duration.nanosec = int(nanoseconds)
    return duration


def retime_trajectory(
    trajectory: RobotTrajectory,
    max_joint_velocity: float,
    min_segment_duration: float,
    time_scale: float,
) -> None:
    points = trajectory.joint_trajectory.points
    if len(points) < 2:
        return
    if max_joint_velocity <= 0.0:
        raise ValueError("'max_joint_velocity' must be > 0.")
    if min_segment_duration <= 0.0:
        raise ValueError("'min_segment_duration' must be > 0.")
    if time_scale <= 0.0:
        raise ValueError("'time_scale' must be > 0.")

    elapsed = 0.0
    previous_positions = list(points[0].positions)
    points[0].time_from_start = duration_from_seconds(elapsed)
    points[0].velocities = []
    points[0].accelerations = []
    points[0].effort = []

    for point in points[1:]:
        positions = list(point.positions)
        if len(positions) == len(previous_positions):
            max_delta = max(abs(current - previous) for current, previous in zip(positions, previous_positions))
        else:
            max_delta = 0.0

        segment_duration = max(min_segment_duration, max_delta / max_joint_velocity)
        elapsed += segment_duration * time_scale
        point.time_from_start = duration_from_seconds(elapsed)
        point.velocities = []
        point.accelerations = []
        point.effort = []
        previous_positions = positions


def moveit_error_name(error_code: MoveItErrorCodes) -> str:
    for name in dir(MoveItErrorCodes):
        if name.isupper() and getattr(MoveItErrorCodes, name) == error_code.val:
            return name
    return f"UNKNOWN({error_code.val})"


class CartesianTrajectoryExecutor(Node):
    def __init__(self, compute_service: str, execute_action: str) -> None:
        super().__init__("execute_cartesian_trajectory")
        self._cartesian_path_client = self.create_client(GetCartesianPath, compute_service)
        self._execute_trajectory_client = ActionClient(self, ExecuteTrajectory, execute_action)

    def compute_path(
        self,
        *,
        frame_id: str,
        group_name: str,
        link_name: str,
        waypoints: list[Pose],
        max_step: float,
        jump_threshold: float,
        prismatic_jump_threshold: float,
        revolute_jump_threshold: float,
        avoid_collisions: bool,
        service_timeout: float,
    ) -> GetCartesianPath.Response:
        if not self._cartesian_path_client.wait_for_service(timeout_sec=service_timeout):
            raise RuntimeError("Timed out waiting for compute_cartesian_path service.")

        request = GetCartesianPath.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = frame_id
        request.start_state.is_diff = True
        request.group_name = group_name
        request.link_name = link_name
        request.waypoints = waypoints
        request.max_step = max_step
        request.jump_threshold = jump_threshold
        request.prismatic_jump_threshold = prismatic_jump_threshold
        request.revolute_jump_threshold = revolute_jump_threshold
        request.avoid_collisions = avoid_collisions

        future = self._cartesian_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=service_timeout)
        if not future.done():
            raise RuntimeError("Timed out while computing Cartesian path.")
        result = future.result()
        if result is None:
            raise RuntimeError("Cartesian path service returned no result.")
        return result

    def execute_trajectory(
        self,
        trajectory: RobotTrajectory,
        action_timeout: float,
        execution_timeout: float | None,
    ) -> MoveItErrorCodes:
        if not self._execute_trajectory_client.wait_for_server(timeout_sec=action_timeout):
            raise RuntimeError("Timed out waiting for execute_trajectory action server.")

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        send_goal_future = self._execute_trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=action_timeout)
        if not send_goal_future.done():
            raise RuntimeError("Timed out while sending execute_trajectory goal.")

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("execute_trajectory goal was rejected.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=execution_timeout)
        if not result_future.done():
            raise RuntimeError("Timed out while executing trajectory.")

        return result_future.result().result.error_code


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compute and optionally execute a MoveIt Cartesian path from pose waypoints."
    )
    parser.add_argument("-f", "--trajectory-file", type=Path, help="YAML or JSON file containing waypoints.")
    parser.add_argument(
        "--waypoint",
        action="append",
        nargs=7,
        type=float,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        help="Add one waypoint directly. Can be repeated.",
    )
    parser.add_argument("--frame-id", help=f"Waypoint frame. Default: {DEFAULT_FRAME_ID}")
    parser.add_argument("--group-name", help=f"MoveIt planning group. Default: {DEFAULT_GROUP_NAME}")
    parser.add_argument("--link-name", help=f"End-effector link for waypoints. Default: {DEFAULT_LINK_NAME}")
    parser.add_argument("--max-step", type=float, help=f"Cartesian interpolation step in meters. Default: {DEFAULT_MAX_STEP}")
    parser.add_argument("--jump-threshold", type=float, help="Relative joint jump threshold. Default: 0.0")
    parser.add_argument("--prismatic-jump-threshold", type=float, help="Absolute prismatic jump threshold. Default: 0.0")
    parser.add_argument("--revolute-jump-threshold", type=float, help="Absolute revolute jump threshold. Default: 0.0")
    parser.add_argument(
        "--avoid-collisions",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Enable or disable collision checking. Default: true",
    )
    parser.add_argument("--min-fraction", type=float, help=f"Required planned path fraction. Default: {DEFAULT_MIN_FRACTION}")
    parser.add_argument("--plan-only", action="store_true", help="Compute the Cartesian path without executing it.")
    parser.add_argument("--no-retime", action="store_true", help="Do not overwrite trajectory timing.")
    parser.add_argument(
        "--max-joint-velocity",
        type=float,
        help=f"Conservative retiming limit in rad/s. Default: {DEFAULT_MAX_JOINT_VELOCITY}",
    )
    parser.add_argument(
        "--min-segment-duration",
        type=float,
        help=f"Minimum duration between returned trajectory points. Default: {DEFAULT_MIN_SEGMENT_DURATION}",
    )
    parser.add_argument("--time-scale", type=float, default=1.0, help="Multiplier applied to retimed durations.")
    parser.add_argument("--compute-service", default="/compute_cartesian_path", help="MoveIt Cartesian path service.")
    parser.add_argument("--execute-action", default="/execute_trajectory", help="MoveIt execute trajectory action.")
    parser.add_argument("--service-timeout", type=float, default=10.0, help="Seconds to wait for services/responses.")
    parser.add_argument("--action-timeout", type=float, default=10.0, help="Seconds to wait for action server/goal accept.")
    parser.add_argument(
        "--execution-timeout",
        type=float,
        default=0.0,
        help="Seconds to wait for execution result. 0 means wait indefinitely.",
    )
    return parser


def value_from_args_or_file(args: argparse.Namespace, data: dict[str, Any], key: str, default: Any) -> Any:
    cli_value = getattr(args, key)
    if cli_value is not None:
        return cli_value
    return data.get(key, default)


def main() -> int:
    args = build_parser().parse_args()
    data = load_trajectory_file(args.trajectory_file) if args.trajectory_file else {}

    waypoint_values = args.waypoint if args.waypoint else data.get("waypoints")
    if not waypoint_values:
        raise SystemExit("No waypoints provided. Use --trajectory-file or at least one --waypoint.")

    try:
        waypoints = [pose_from_waypoint(waypoint) for waypoint in waypoint_values]
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    frame_id = value_from_args_or_file(args, data, "frame_id", DEFAULT_FRAME_ID)
    group_name = value_from_args_or_file(args, data, "group_name", DEFAULT_GROUP_NAME)
    link_name = value_from_args_or_file(args, data, "link_name", DEFAULT_LINK_NAME)
    max_step = float(value_from_args_or_file(args, data, "max_step", DEFAULT_MAX_STEP))
    jump_threshold = float(value_from_args_or_file(args, data, "jump_threshold", 0.0))
    prismatic_jump_threshold = float(value_from_args_or_file(args, data, "prismatic_jump_threshold", 0.0))
    revolute_jump_threshold = float(value_from_args_or_file(args, data, "revolute_jump_threshold", 0.0))
    avoid_collisions = bool(value_from_args_or_file(args, data, "avoid_collisions", True))
    min_fraction = float(value_from_args_or_file(args, data, "min_fraction", DEFAULT_MIN_FRACTION))
    max_joint_velocity = float(value_from_args_or_file(args, data, "max_joint_velocity", DEFAULT_MAX_JOINT_VELOCITY))
    min_segment_duration = float(
        value_from_args_or_file(args, data, "min_segment_duration", DEFAULT_MIN_SEGMENT_DURATION)
    )

    if max_step <= 0.0:
        raise SystemExit("--max-step must be > 0.")
    if not 0.0 <= min_fraction <= 1.0:
        raise SystemExit("--min-fraction must be between 0 and 1.")

    rclpy.init()
    node = CartesianTrajectoryExecutor(args.compute_service, args.execute_action)
    try:
        node.get_logger().info(
            f"Computing Cartesian path for {len(waypoints)} waypoint(s), group='{group_name}', "
            f"link='{link_name}', frame='{frame_id}'"
        )
        response = node.compute_path(
            frame_id=frame_id,
            group_name=group_name,
            link_name=link_name,
            waypoints=waypoints,
            max_step=max_step,
            jump_threshold=jump_threshold,
            prismatic_jump_threshold=prismatic_jump_threshold,
            revolute_jump_threshold=revolute_jump_threshold,
            avoid_collisions=avoid_collisions,
            service_timeout=args.service_timeout,
        )

        error_name = moveit_error_name(response.error_code)
        node.get_logger().info(f"Cartesian path result: fraction={response.fraction:.3f}, error={error_name}")
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            node.get_logger().error(f"MoveIt failed to compute the path: {error_name}")
            return 1
        if response.fraction < min_fraction:
            node.get_logger().error(
                f"Only {response.fraction:.3f} of the path was planned; required at least {min_fraction:.3f}."
            )
            return 1

        point_count = len(response.solution.joint_trajectory.points)
        if point_count == 0:
            node.get_logger().error("MoveIt returned an empty trajectory.")
            return 1

        if not args.no_retime:
            retime_trajectory(response.solution, max_joint_velocity, min_segment_duration, args.time_scale)
            node.get_logger().info(
                f"Retimed {point_count} trajectory point(s) with max_joint_velocity={max_joint_velocity:.3f} rad/s."
            )

        if args.plan_only:
            node.get_logger().info("Plan-only mode: trajectory was not executed.")
            return 0

        execution_timeout = None if args.execution_timeout <= 0.0 else args.execution_timeout
        execution_code = node.execute_trajectory(response.solution, args.action_timeout, execution_timeout)
        execution_name = moveit_error_name(execution_code)
        if execution_code.val != MoveItErrorCodes.SUCCESS:
            node.get_logger().error(f"Trajectory execution failed: {execution_name}")
            return 1

        node.get_logger().info("Trajectory execution succeeded.")
        return 0
    except (RuntimeError, ValueError) as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
