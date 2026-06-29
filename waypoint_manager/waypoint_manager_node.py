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

from waypoint_manager.diagnostics_logger import DiagnosticsTxtLogger
from waypoint_manager.leg_loader import (
    DEVICE_NAMES,
    autonomy_origin_for_vertex,
    build_hop_route,
    diagnostic_level_to_int,
    get_target_vertex_from_statuses,
    load_route_waypoints,
    load_vertex_positions,
    map_pose_to_autonomy_frame,
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
        self.declare_parameter("vertices_file", "")
        self.declare_parameter("default_frame_id", "map")
        self.declare_parameter("home_vertex", 1)
        self.declare_parameter("odometry_origin_vertex", 1)
        self.declare_parameter("sync_odometry_origin_on_vertex_set", False)
        self.declare_parameter("auto_start", True)

        # autonomy_stack_go2 (https://github.com/jizhang-cmu/autonomy_stack_go2)
        self.declare_parameter("way_point_topic", "/way_point")
        self.declare_parameter("state_estimation_topic", "/state_estimation")
        self.declare_parameter("speed_topic", "/speed")
        self.declare_parameter("navigation_speed", 1.0)
        self.declare_parameter("waypoint_xy_radius", 0.5)
        self.declare_parameter("waypoint_z_bound", 5.0)
        self.declare_parameter("waypoint_publish_rate", 5.0)
        self.declare_parameter("diagnostics_log_enabled", True)
        self.declare_parameter(
            "diagnostics_log_path",
            str(Path.home() / "waypoint_manager" / "logs" / "diagnostics_log.txt"),
        )

        self._frame_id = str(self.get_parameter("default_frame_id").value)
        self._home_vertex = int(self.get_parameter("home_vertex").value)
        self._odometry_origin_vertex = int(
            self.get_parameter("odometry_origin_vertex").value
        )
        self._sync_origin_on_vertex_set = bool(
            self.get_parameter("sync_odometry_origin_on_vertex_set").value
        )
        self._auto_start = bool(self.get_parameter("auto_start").value)
        self._legs_dir = self._resolve_legs_dir()
        self._vertices_file = self._resolve_vertices_file()
        self._vertex_positions: Dict[int, Tuple[float, float]] = {}
        self._load_vertex_config()

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

        self._diagnostics_logger: Optional[DiagnosticsTxtLogger] = None
        if bool(self.get_parameter("diagnostics_log_enabled").value):
            log_path = Path(str(self.get_parameter("diagnostics_log_path").value)).expanduser()
            try:
                self._diagnostics_logger = DiagnosticsTxtLogger(log_path)
            except OSError as exc:
                self.get_logger().error(f"Failed to open diagnostics text log: {exc}")

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

        self.create_subscription(
            Int32, "~/set_current_vertex", self._on_set_current_vertex, 10
        )
        self.create_subscription(
            Int32, "~/set_odometry_origin", self._on_set_odometry_origin, 10
        )

        period = 1.0 / publish_rate if publish_rate > 0.0 else 0.2
        self.create_timer(period, self._navigation_tick)

        self._publish_current_vertex()
        self.get_logger().info(
            f"Waypoint manager ready (autonomy_stack_go2). "
            f"diagnostics={diagnostics_topic}, way_point={way_point_topic}, "
            f"odom={odom_topic}, legs_dir={self._legs_dir}"
        )
        if self._diagnostics_logger is not None:
            self.get_logger().info(
                f"Diagnostics text log: {self._diagnostics_logger.path}"
            )
        ox, oy = self._autonomy_origin_xy()
        self.get_logger().info(
            f"Waypoint frame: map -> autonomy via vertex "
            f"{self._odometry_origin_vertex} offset=({ox:.3f}, {oy:.3f})"
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

    def _resolve_vertices_file(self) -> Path:
        vertices_param = str(self.get_parameter("vertices_file").value).strip()
        if vertices_param:
            return Path(vertices_param).expanduser()

        try:
            from ament_index_python.packages import get_package_share_directory

            share = get_package_share_directory("waypoint_manager")
            return Path(share) / "config" / "vertices.yaml"
        except Exception:
            return Path(__file__).resolve().parent.parent / "config" / "vertices.yaml"

    def _load_vertex_config(self) -> None:
        try:
            frame_id, positions = load_vertex_positions(self._vertices_file)
            self._vertex_positions = positions
            if frame_id:
                self._frame_id = frame_id
        except (OSError, ValueError, RuntimeError, KeyError) as exc:
            self.get_logger().error(
                f"Failed to load vertices from {self._vertices_file}: {exc}"
            )
            self._vertex_positions = {}

    def _autonomy_origin_xy(self) -> Tuple[float, float]:
        if not self._vertex_positions:
            return 0.0, 0.0
        try:
            return autonomy_origin_for_vertex(
                self._vertex_positions, self._odometry_origin_vertex
            )
        except KeyError:
            return 0.0, 0.0

    def _map_to_autonomy_xy(self, map_x: float, map_y: float) -> Tuple[float, float]:
        origin_x, origin_y = self._autonomy_origin_xy()
        return map_pose_to_autonomy_frame(
            map_x, map_y, origin_x=origin_x, origin_y=origin_y
        )

    def _on_set_current_vertex(self, msg: Int32) -> None:
        if self._nav_state == NavState.NAVIGATING:
            self.get_logger().warn(
                "Cannot set current_vertex while navigating; cancel first."
            )
            return

        vertex = int(msg.data)
        if vertex not in self._vertex_positions:
            self.get_logger().error(f"Invalid current_vertex {vertex}; expected 1-4.")
            return

        self._current_vertex = vertex
        self._publish_current_vertex()
        self.get_logger().info(f"Current vertex set to {vertex} (skip/manual).")

        if self._sync_origin_on_vertex_set:
            self._set_odometry_origin_vertex(vertex)

    def _on_set_odometry_origin(self, msg: Int32) -> None:
        self._set_odometry_origin_vertex(int(msg.data))

    def _set_odometry_origin_vertex(self, vertex: int) -> None:
        if vertex not in self._vertex_positions:
            self.get_logger().error(
                f"Invalid odometry_origin_vertex {vertex}; expected 1-4."
            )
            return

        self._odometry_origin_vertex = vertex
        ox, oy = self._autonomy_origin_xy()
        self.get_logger().info(
            f"Odometry origin vertex set to {vertex}; "
            f"map offset=({ox:.3f}, {oy:.3f}). "
            "Use after autonomy_stack_go2 restarts at that vertex."
        )
        if self._nav_state == NavState.IDLE and self._waypoints:
            self._publish_markers()

    def _on_odometry(self, msg: Odometry) -> None:
        self._vehicle_x = float(msg.pose.pose.position.x)
        self._vehicle_y = float(msg.pose.pose.position.y)
        self._vehicle_z = float(msg.pose.pose.position.z)
        self._have_pose = True

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        if self._diagnostics_logger is not None:
            stamp = msg.header.stamp
            if stamp.sec or stamp.nanosec:
                timestamp = f"{stamp.sec}.{stamp.nanosec:09d}"
            else:
                timestamp = self.get_clock().now().to_msg()
                timestamp = f"{timestamp.sec}.{timestamp.nanosec:09d}"
            self._diagnostics_logger.log_array(msg, timestamp)

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
        ox, oy = self._autonomy_origin_xy()
        self.get_logger().info(
            f"Starting hop: {' -> '.join(str(v) for v in self._route)} "
            f"(from vertex {self._current_vertex}, levels={self._device_levels}, "
            f"autonomy origin vertex {self._odometry_origin_vertex} "
            f"offset=({ox:.3f}, {oy:.3f}))"
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
        wp_x, wp_y = self._map_to_autonomy_xy(
            wp.pose.position.x, wp.pose.position.y
        )
        dx = self._vehicle_x - wp_x
        dy = self._vehicle_y - wp_y
        dz = self._vehicle_z - wp.pose.position.z
        dist_xy = math.hypot(dx, dy)

        if dist_xy < self._waypoint_xy_radius and abs(dz) < self._waypoint_z_bound:
            self.get_logger().info(
                f"Reached leg waypoint {self._wp_index + 1}/{len(self._waypoints)} "
                f"map=({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f}) "
                f"autonomy=({wp_x:.2f}, {wp_y:.2f})"
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
        wp_x, wp_y = self._map_to_autonomy_xy(
            wp.pose.position.x, wp.pose.position.y
        )
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = wp.header.frame_id or self._frame_id
        msg.point.x = wp_x
        msg.point.y = wp_y
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
            wp_x, wp_y = self._map_to_autonomy_xy(
                wp.pose.position.x, wp.pose.position.y
            )
            marker = Marker()
            marker.header.frame_id = wp.header.frame_id or self._frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoint_manager"
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = wp.pose
            marker.pose.position.x = wp_x
            marker.pose.position.y = wp_y
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
        if node._diagnostics_logger is not None:
            node._diagnostics_logger.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
