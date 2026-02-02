#  Copyright (c) 2026 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_share = get_package_share_directory('franka_bringup')
utils_path = os.path.abspath(
    os.path.join(package_share, '..', '..', 'lib', 'franka_bringup', 'utils')
)
launch_utils_path = os.path.join(utils_path, 'launch_utils.py')

spec = importlib.util.spec_from_file_location('launch_utils', launch_utils_path)
launch_utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_utils)

load_yaml = launch_utils.load_yaml


def generate_robot_nodes(context):
    additional_nodes = []
    robot_config_file = LaunchConfiguration('robot_config_file').perform(context)

    additional_nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('franka_bringup'), 'launch', 'example.launch.py'
                ])
            ),
            launch_arguments={
                'robot_config_file': robot_config_file,
                'controller_names': 'exercise_cartesian_impedance_controller',
                'rviz_config_file': PathJoinSubstitution([
                    FindPackageShare('franka_bringup'),
                    'rviz',
                    'visualize_franka_with_interactive_marker.rviz',
                ]),
            }.items(),
        )
    )

    configs = load_yaml(robot_config_file)

    for _, config in configs.items():
        namespace = config['namespace']
        robot_type = config['robot_type']
        base_link = 'base'
        ee_link = f'{robot_type}_hand_tcp'

        additional_nodes.append(
            Node(
                package='franka_simple_publishers',
                executable='simple_interactive_marker_pose_publisher',
                namespace=namespace,
                output='screen',
                parameters=[
                    {'topic_name': '/cartesian_impedance/desired_pose'},
                    {'base_link': base_link},
                    {'ee_link': ee_link},
                    {'transition_event_topic': 'exercise_cartesian_impedance_controller/transition_event'},
                ],
            )
        )

    return additional_nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_bringup'), 'config', 'franka.config.yaml'
            ]),
            description='Path to the robot configuration file to load',
        ),
        OpaqueFunction(function=generate_robot_nodes),
    ])
