Requisitos previos:

Tener instalado ROS 2.

Tener instalado colcon para compilar el workspace.

Haber configurado las variables de entorno de ROS 2.

Instalación y uso:

    Clonar el repositorio:

    1.Clonar el repositorio.

    git clone https://github.com/rbriz758/g12_prii3

    2.Entrar al workspace.

    cd g12_prii3_ws

    3.Compilar el workspace con colcon.

    colcon build

    4.Iniciar el entorno.
    source install/setup.bash

    5. Lanzar el nodo con el launch file:

    ros2 launch g12_prii3_turtlesim mover_tortuga.launch.py

    6.Servicios Parar/Reunudar/Reiniciar:

    Parar: ros2 service call /detener_dibujo std_srvs/srv/Trigger {}

    Reunudar: ros2 service call /reanudar_dibujo std_srvs/srv/Trigger {}

    Reiniciar: ros2 service call /reiniciar_dibujo std_srvs/srv/Trigger {}


