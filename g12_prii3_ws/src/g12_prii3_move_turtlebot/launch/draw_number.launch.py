from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g12_prii3_move_turtlebot',
            executable='draw_number',
            name='draw_number',
            output='screen'
        )
    ])
