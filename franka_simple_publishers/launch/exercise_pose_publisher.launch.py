from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch argument (string, will be parsed as YAML list)
    goal_arg = DeclareLaunchArgument(
        "goal",
        default_value="[5.0, 0.65, 0.25, 0.35, 0.5, 0.5, 0.5, 0.5]",
        description="Goal as [T, x, y, z, qx, qy, qz, qw]"
    )

    return LaunchDescription([
        goal_arg,

        Node(
            package="franka_simple_publishers",
            executable="exercise_pose_publisher",
            name="exercise_pose_publisher",
            output="screen",
            parameters=[
                {
                    "goal": LaunchConfiguration("goal"),
                }
            ],
        )
    ])
