from setuptools import setup
import os
from glob import glob

package_name = 'g12_prii3_move_turtlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='isaac',
    maintainer_email='ipermas@upv.edu.es',
    description='Nodo de movimiento y evitación para TurtleBot3 en Gazebo.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'draw_number = g12_prii3_move_turtlebot.draw_number:main',
            'collision_avoidance = g12_prii3_move_turtlebot.collision_avoidance:main',
            'obstacle_avoidance = g12_prii3_move_turtlebot.obstacle_avoidance:main',
        ],
    },
)
