from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo del simulador turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='simulador_tortuga'
        ),
        # Nodo personalizado para mover la tortuga
        Node(
            package='g12_prii3_turtlesim',
            executable='mover_tortuga',
            name='mover_tortuga'
        )
    ])