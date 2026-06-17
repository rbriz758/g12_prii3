import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_g12_prii3 = get_package_share_directory('g12_prii3_nav_turtlebot')
    pkg_turtlebot3_nav2 = get_package_share_directory('turtlebot3_navigation2')

    # 1. Lanzar Gazebo + Robot State Publisher + Spawn (f1l3_world.launch.py)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_g12_prii3, 'launch', 'f1l3_world.launch.py')
        )
    )

    # 2. Lanzar Navigation2 (Map Server, AMCL, Planner, Controller, BT, etc.)
    map_file = os.path.join(pkg_g12_prii3, 'maps', 'map.yaml')
    
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_nav2, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true'
        }.items()
    )

    # 3. Lanzar el nodo de navegación predefinida
    predefined_node = Node(
        package='g12_prii3_nav_turtlebot',
        executable='predefined_nav',
        name='predefined_nav_node',
        output='screen'
    )

    return LaunchDescription([
        gazebo_sim,
        navigation,
        # Damos unos segundos para que Gazebo y Nav2 arranquen
        TimerAction(
            period=5.0,
            actions=[predefined_node]
        )
    ])
