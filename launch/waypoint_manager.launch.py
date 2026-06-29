from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("waypoint_manager")

    default_legs = PathJoinSubstitution([pkg, "config", "legs"])
    default_vertices = PathJoinSubstitution([pkg, "config", "vertices.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "legs_dir",
            default_value=default_legs,
            description="Directory containing {from}_to_{to}.yaml leg files.",
        ),
        DeclareLaunchArgument(
            "vertices_file",
            default_value=default_vertices,
            description="YAML with map-frame coordinates for vertices 1-4.",
        ),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("odometry_origin_vertex", default_value="1"),
        DeclareLaunchArgument(
            "sync_odometry_origin_on_vertex_set",
            default_value="false",
        ),
        DeclareLaunchArgument("diagnostics_log_enabled", default_value="true"),
        DeclareLaunchArgument(
            "way_point_topic",
            default_value="/waypoint_manager/target_waypoint",
            description="Waypoint_Manager target topic consumed by run_relative_waypoint_sequence.py.",
        ),
        DeclareLaunchArgument(
            "speed_topic",
            default_value="/speed",
            description="autonomy_stack_go2 speed topic.",
        ),
        Node(
            package="waypoint_manager",
            executable="waypoint_manager_node",
            name="waypoint_manager",
            output="screen",
            parameters=[{
                "legs_dir": LaunchConfiguration("legs_dir"),
                "vertices_file": LaunchConfiguration("vertices_file"),
                "auto_start": LaunchConfiguration("auto_start"),
                "odometry_origin_vertex": LaunchConfiguration(
                    "odometry_origin_vertex"
                ),
                "sync_odometry_origin_on_vertex_set": LaunchConfiguration(
                    "sync_odometry_origin_on_vertex_set"
                ),
                "diagnostics_log_enabled": LaunchConfiguration(
                    "diagnostics_log_enabled"
                ),
                "way_point_topic": LaunchConfiguration("way_point_topic"),
                "speed_topic": LaunchConfiguration("speed_topic"),
            }],
        ),
    ])
