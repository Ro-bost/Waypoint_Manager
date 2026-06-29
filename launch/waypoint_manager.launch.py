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
        DeclareLaunchArgument(
            "home_vertex",
            default_value="1",
            description="Initial logical vertex where the robot starts.",
        ),
        DeclareLaunchArgument("odometry_origin_vertex", default_value="1"),
        DeclareLaunchArgument("diagnostics_log_enabled", default_value="true"),
        DeclareLaunchArgument(
            "way_point_topic",
            default_value="/waypoint_manager/target_waypoint",
            description="Waypoint_Manager target topic consumed by run_relative_waypoint_sequence.py.",
        ),
        DeclareLaunchArgument(
            "waypoint_reached_topic",
            default_value="/waypoint_manager/waypoint_reached",
            description="Int32 completion signal topic; publish data=1 to advance to the next waypoint.",
        ),
        Node(
            package="waypoint_manager",
            executable="waypoint_manager_node",
            name="waypoint_manager",
            output="screen",
            parameters=[{
                "legs_dir": LaunchConfiguration("legs_dir"),
                "vertices_file": LaunchConfiguration("vertices_file"),
                "home_vertex": LaunchConfiguration("home_vertex"),
                "odometry_origin_vertex": LaunchConfiguration(
                    "odometry_origin_vertex"
                ),
                "diagnostics_log_enabled": LaunchConfiguration(
                    "diagnostics_log_enabled"
                ),
                "way_point_topic": LaunchConfiguration("way_point_topic"),
                "waypoint_reached_topic": LaunchConfiguration(
                    "waypoint_reached_topic"
                ),
            }],
        ),
    ])
