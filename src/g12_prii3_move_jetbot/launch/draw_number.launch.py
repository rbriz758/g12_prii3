#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # Set TURTLEBOT3_MODEL environment variable for compatibility with turtlebot3_gazebo
    set_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger')

    # Include TurtleBot3 empty world gazebo launch
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_gazebo_pkg, 'launch', 'empty_world.launch.py'))
    )

    draw_node = Node(
        package='g12_prii3_move_jetbot',
        executable='draw_number',
        name='draw_number',
        output='screen'
    )

    return LaunchDescription([
        set_model,
        empty_world,
        draw_node,
    ])
