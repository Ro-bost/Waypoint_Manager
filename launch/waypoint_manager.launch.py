from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("waypoint_manager")

    legs_dir = LaunchConfiguration("legs_dir")
    auto_start = LaunchConfiguration("auto_start")

    default_legs = PathJoinSubstitution([pkg, "config", "legs"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "legs_dir",
            default_value=default_legs,
            description="Directory containing {from}_to_{to}.yaml leg files.",
        ),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument(
            "way_point_topic",
            default_value="/way_point",
            description="autonomy_stack_go2 waypoint topic.",
        ),
        DeclareLaunchArgument(
            "state_estimation_topic",
            default_value="/state_estimation",
            description="autonomy_stack_go2 odometry topic.",
        ),
        Node(
            package="waypoint_manager",
            executable="waypoint_manager_node",
            name="waypoint_manager",
            output="screen",
            parameters=[{
                "legs_dir": legs_dir,
                "auto_start": auto_start,
                "way_point_topic": LaunchConfiguration("way_point_topic"),
                "state_estimation_topic": LaunchConfiguration("state_estimation_topic"),
            }],
        ),
    ])
