from setuptools import setup
import os
from glob import glob

package_name = 'g12_prii3_nav_turtlebot'

def package_files(directory_list):
    paths = []
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                paths.append((install_path, [file_path]))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # LÍNEA AÑADIDA: Copia todos los ficheros .world de la carpeta worlds/
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        
        # LÍNEA MODIFICADA: Asegura que todos los ficheros .py de la carpeta launch/ se copien
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ] + package_files(['models', 'maps']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='isaac',
    maintainer_email='ipermas@upv.edu.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_navigation = g12_prii3_nav_turtlebot.autonomous_navigation:main',
            'predefined_nav = g12_prii3_nav_turtlebot.predefined_nav_node:main',
        ],
    },
)