"""Launch diffdrive_bot simulation, rotate action server, and square driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_diffdrive = get_package_share_directory('diffdrive_bot')

    side_length = ParameterValue(LaunchConfiguration('side_length'), value_type=float)

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_diffdrive, 'launch', 'control.launch.py')
        )
    )

    rotate_server_node = Node(
        package='my_robot_nodes',
        executable='rotate_server',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    square_driver_node = Node(
        package='my_robot_nodes',
        executable='square_driver',
        parameters=[{'side_length': side_length, 'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'side_length',
            default_value='0.5',
            description='Side length of the square in meters',
        ),
        control_launch,
        rotate_server_node,
        square_driver_node,
    ])
