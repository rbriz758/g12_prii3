import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Rutas y configuraciones
    # ---
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_name = 'g12_prii3_nav_turtlebot' # Tu paquete

    # Configurar modelo por defecto a WAFFLE (Requisito Sprint 3)
    if 'TURTLEBOT3_MODEL' not in os.environ:
        os.environ['TURTLEBOT3_MODEL'] = 'waffle'

    # Añadir modelos de turtlebot3 al path de Gazebo
    model_path = os.path.join(pkg_turtlebot3_gazebo, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ':' + model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = model_path

    # Argumentos de configuración
    # Asegúrate de que el nombre aquí coincide con tu archivo real (f1f3.world o f1l3.world)
    world_file_arg = LaunchConfiguration('world_file', default='f1f3.world')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    


    # Ruta al archivo del mundo
    world = PathJoinSubstitution([
        get_package_share_directory(pkg_name),
        'worlds',
        world_file_arg
    ])

    # 2. Nodos y Lanzamientos
    
    # Lanzar Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
            )
        ]),
        launch_arguments={'world': world}.items()
    )

    # Publicador de estado del robot (Lee el URDF del Waffle)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )



    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'world_file',
            default_value='f1f3.world', 
            description='Name of the world file located in the package worlds/ directory'),



        gazebo,
    robot_state_publisher,
    
    Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-database', 'turtlebot3_waffle',
                   '-entity', 'turtlebot3_waffle',
                   '-x', '-9.0',
                   '-y', '-4.0',
                   '-z', '0.01'],
        output='screen'
    ),

    ])