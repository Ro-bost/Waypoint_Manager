#!/usr/bin/env python3
"""Factory diagnostics -> leg YAMLs -> autonomy_stack_go2 /way_point navigation."""

from __future__ import annotations

import math
from enum import Enum
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Int32, Int32MultiArray
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from waypoint_manager.leg_loader import (
    DEVICE_NAMES,
    build_hop_route,
    diagnostic_level_to_int,
    get_target_vertex_from_statuses,
    load_route_waypoints,
    parse_vertex_from_name,
    route_to_leg_pairs,
)


class NavState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"


class WaypointManagerNode(Node):
    """ICROS diagnostics-driven waypoint manager for autonomy_stack_go2."""

    def __init__(self) -> None:
        super().__init__("waypoint_manager")

        self.declare_parameter("diagnostics_topic", "/factory/diagnostics")
        self.declare_parameter("legs_dir", "")
        self.declare_parameter("default_frame_id", "map")
        self.declare_parameter("home_vertex", 1)
        self.declare_parameter("auto_start", True)

        # autonomy_stack_go2 (https://github.com/jizhang-cmu/autonomy_stack_go2)
        self.declare_parameter("way_point_topic", "/way_point")
        self.declare_parameter("state_estimation_topic", "/state_estimation")
        self.declare_parameter("speed_topic", "/speed")
        self.declare_parameter("navigation_speed", 1.0)
        self.declare_parameter("waypoint_xy_radius", 0.5)
        self.declare_parameter("waypoint_z_bound", 5.0)
        self.declare_parameter("waypoint_publish_rate", 5.0)

        self._frame_id = str(self.get_parameter("default_frame_id").value)
        self._home_vertex = int(self.get_parameter("home_vertex").value)
        self._auto_start = bool(self.get_parameter("auto_start").value)
        self._legs_dir = self._resolve_legs_dir()

        self._waypoint_xy_radius = float(self.get_parameter("waypoint_xy_radius").value)
        self._waypoint_z_bound = float(self.get_parameter("waypoint_z_bound").value)
        self._navigation_speed = float(self.get_parameter("navigation_speed").value)
        publish_rate = float(self.get_parameter("waypoint_publish_rate").value)

        self._device_levels: Dict[str, int] = {name: 0 for name in DEVICE_NAMES}
        self._last_statuses: list = []
        self._nav_state = NavState.IDLE
        self._current_vertex = self._home_vertex
        self._mission_target: Optional[int] = None
        self._route: List[int] = [self._home_vertex]
        self._waypoints: List[PoseStamped] = []
        self._wp_index = 0

        self._vehicle_x = 0.0
        self._vehicle_y = 0.0
        self._vehicle_z = 0.0
        self._have_pose = False

        diag_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        marker_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self.create_subscription(
            DiagnosticArray, diagnostics_topic, self._on_diagnostics, diag_qos
        )

        odom_topic = str(self.get_parameter("state_estimation_topic").value)
        self.create_subscription(Odometry, odom_topic, self._on_odometry, 10)

        way_point_topic = str(self.get_parameter("way_point_topic").value)
        speed_topic = str(self.get_parameter("speed_topic").value)
        self._way_point_pub = self.create_publisher(PointStamped, way_point_topic, 5)
        self._speed_pub = self.create_publisher(Float32, speed_topic, 5)

        self._route_pub = self.create_publisher(Int32MultiArray, "~/route", 10)
        self._current_vertex_pub = self.create_publisher(Int32, "~/current_vertex", 10)
        self._arrival_pub = self.create_publisher(Int32, "~/arrival", 10)
        self._marker_pub = self.create_publisher(MarkerArray, "~/markers", marker_qos)
        self._queue_pub = self.create_publisher(Int32, "~/waypoint_count", 10)

        self.create_service(Trigger, "~/start", self._on_start)
        self.create_service(Trigger, "~/cancel", self._on_cancel)
        self.create_service(Trigger, "~/reload", self._on_reload)

        period = 1.0 / publish_rate if publish_rate > 0.0 else 0.2
        self.create_timer(period, self._navigation_tick)

        self._publish_current_vertex()
        self.get_logger().info(
            f"Waypoint manager ready (autonomy_stack_go2). "
            f"diagnostics={diagnostics_topic}, way_point={way_point_topic}, "
            f"odom={odom_topic}, legs_dir={self._legs_dir}"
        )

    def _resolve_legs_dir(self) -> Path:
        legs_dir_param = str(self.get_parameter("legs_dir").value).strip()
        if legs_dir_param:
            return Path(legs_dir_param).expanduser()

        try:
            from ament_index_python.packages import get_package_share_directory

            share = get_package_share_directory("waypoint_manager")
            return Path(share) / "config" / "legs"
        except Exception:
            return Path(__file__).resolve().parent.parent / "config" / "legs"

    def _on_odometry(self, msg: Odometry) -> None:
        self._vehicle_x = float(msg.pose.pose.position.x)
        self._vehicle_y = float(msg.pose.pose.position.y)
        self._vehicle_z = float(msg.pose.pose.position.z)
        self._have_pose = True

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            level = diagnostic_level_to_int(status.level)
            if status.name in self._device_levels:
                self._device_levels[status.name] = level
            elif level != 0:
                vertex = parse_vertex_from_name(status.name)
                self.get_logger().warn(
                    f"Non-OK status with unmapped name '{status.name}' "
                    f"(level={level}, vertex={vertex}); ignored."
                )

        self._last_statuses = list(msg.status)

        # Only start a new hop when idle (previous navigation finished).
        if self._nav_state != NavState.IDLE:
            self.get_logger().debug(
                "Navigation in progress; diagnostics stored for after arrival.",
                throttle_duration_sec=5.0,
            )
            return

        if self._auto_start:
            self._try_start_next_hop(self._last_statuses)

    def _try_start_next_hop(self, statuses: list) -> bool:
        if self._nav_state != NavState.IDLE:
            return False

        target = get_target_vertex_from_statuses(statuses)
        if target is None:
            return False

        if target == self._current_vertex:
            self.get_logger().debug(
                f"Already at vertex {self._current_vertex}; waiting for next target.",
                throttle_duration_sec=10.0,
            )
            return False

        hop_route = build_hop_route(self._current_vertex, target)
        if not route_to_leg_pairs(hop_route):
            return False

        self._route = hop_route
        self._mission_target = target
        self._publish_route()

        if not self._load_route_waypoints():
            self._mission_target = None
            return False

        self._wp_index = 0
        self._nav_state = NavState.NAVIGATING
        self.get_logger().info(
            f"Starting hop: {' -> '.join(str(v) for v in self._route)} "
            f"(from vertex {self._current_vertex}, levels={self._device_levels})"
        )
        return True

    def _navigation_tick(self) -> None:
        if self._nav_state != NavState.NAVIGATING or not self._waypoints:
            return

        if self._wp_index >= len(self._waypoints):
            self._complete_hop()
            return

        self._publish_active_waypoint()
        self._publish_speed()

        if not self._have_pose:
            return

        wp = self._waypoints[self._wp_index]
        dx = self._vehicle_x - wp.pose.position.x
        dy = self._vehicle_y - wp.pose.position.y
        dz = self._vehicle_z - wp.pose.position.z
        dist_xy = math.hypot(dx, dy)

        if dist_xy < self._waypoint_xy_radius and abs(dz) < self._waypoint_z_bound:
            self.get_logger().info(
                f"Reached leg waypoint {_wp_index + 1}/{len(self._waypoints)} "
                f"({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f})"
            )
            self._wp_index += 1
            if self._wp_index >= len(self._waypoints):
                self._complete_hop()

    def _complete_hop(self) -> None:
        if self._mission_target is not None:
            self._current_vertex = self._mission_target
            self._publish_current_vertex()

            arrival = Int32()
            arrival.data = int(self._current_vertex)
            self._arrival_pub.publish(arrival)

            self.get_logger().info(
                f"Hop complete. Arrived at vertex {self._current_vertex}."
            )

        self._nav_state = NavState.IDLE
        self._mission_target = None
        self._wp_index = 0

        # Process diagnostics that may have arrived during navigation.
        if self._auto_start and self._last_statuses:
            self._try_start_next_hop(self._last_statuses)

    def _publish_active_waypoint(self) -> None:
        if self._wp_index >= len(self._waypoints):
            return

        wp = self._waypoints[self._wp_index]
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = wp.header.frame_id or self._frame_id
        msg.point.x = wp.pose.position.x
        msg.point.y = wp.pose.position.y
        msg.point.z = wp.pose.position.z
        self._way_point_pub.publish(msg)

    def _publish_speed(self) -> None:
        speed = Float32()
        speed.data = float(self._navigation_speed)
        self._speed_pub.publish(speed)

    def _load_route_waypoints(self) -> bool:
        pairs = route_to_leg_pairs(self._route)
        if not pairs:
            self._waypoints = []
            self._publish_markers()
            self._publish_waypoint_count()
            return False

        try:
            frame_id, poses = load_route_waypoints(
                self._route, self._legs_dir, self._frame_id
            )
        except (FileNotFoundError, ValueError, RuntimeError) as exc:
            self.get_logger().error(f"Failed to load route waypoints: {exc}")
            self._waypoints = []
            self._publish_markers()
            self._publish_waypoint_count()
            return False

        self._waypoints = poses
        self._frame_id = frame_id
        self._publish_markers()
        self._publish_waypoint_count()
        self.get_logger().info(
            f"Loaded {len(self._waypoints)} waypoint(s) for hop "
            f"{' -> '.join(str(v) for v in self._route)}"
        )
        return True

    def _on_start(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self._nav_state == NavState.NAVIGATING:
            response.success = False
            response.message = "Mission already running."
            return response

        if self._last_statuses:
            started = self._try_start_next_hop(self._last_statuses)
        elif self._load_route_waypoints():
            self._wp_index = 0
            self._nav_state = NavState.NAVIGATING
            started = True
        else:
            started = False

        response.success = started
        response.message = (
            "Navigation started." if started else "No hop available to start."
        )
        return response

    def _on_cancel(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self._nav_state != NavState.NAVIGATING:
            response.success = False
            response.message = "No active navigation."
            return response
        self._nav_state = NavState.IDLE
        self._mission_target = None
        self._wp_index = 0
        response.success = True
        response.message = "Navigation cancelled."
        return response

    def _on_reload(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self._nav_state == NavState.NAVIGATING:
            response.success = False
            response.message = "Navigation running."
            return response
        ok = self._load_route_waypoints()
        response.success = ok
        response.message = (
            f"Reloaded {len(self._waypoints)} waypoint(s)."
            if ok
            else "Reload failed."
        )
        return response

    def _publish_route(self) -> None:
        msg = Int32MultiArray()
        msg.data = [int(v) for v in self._route]
        self._route_pub.publish(msg)

    def _publish_current_vertex(self) -> None:
        msg = Int32()
        msg.data = int(self._current_vertex)
        self._current_vertex_pub.publish(msg)

    def _publish_waypoint_count(self) -> None:
        msg = Int32()
        msg.data = len(self._waypoints)
        self._queue_pub.publish(msg)

    def _publish_markers(self) -> None:
        markers = MarkerArray()
        for idx, wp in enumerate(self._waypoints):
            marker = Marker()
            marker.header.frame_id = wp.header.frame_id or self._frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoint_manager"
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = wp.pose
            marker.scale.x = 0.45
            marker.scale.y = 0.08
            marker.scale.z = 0.08
            marker.color.r = 0.95
            marker.color.g = 0.55
            marker.color.b = 0.1
            marker.color.a = 0.95
            markers.markers.append(marker)

        if not self._waypoints:
            delete_all = Marker()
            delete_all.action = Marker.DELETEALL
            markers.markers = [delete_all]

        self._marker_pub.publish(markers)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = WaypointManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
