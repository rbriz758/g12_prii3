from setuptools import setup

package_name = 'g12_prii3_turtlesim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rodrigo',
    maintainer_email='tu_email@dominio.com',
    description='Control de turtlesim para el grupo 12',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mover_tortuga = g12_prii3_turtlesim.mover_tortuga:main',
        ],
    },
)
