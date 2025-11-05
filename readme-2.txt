
## Requisitos previos

- ROS 2 
- `colcon` para construir el workspace
- Dependencias de paquetes instaladas gazebo


## g12_prii3_move_turtlebot

Paquete enfocado a TurtleBot3 en Gazebo. Incluye 3 launch files relevantes:

- `launch/draw_number_only.launch.py` — Arranca Gazebo (mundo vacío) y el nodo
  `draw_number` (trazado del número 12).

- `launch/draw_and_collision.launch.py` — Igual que el anterior más el nodo
  `collision_avoidance` (comportamiento de supervisión de colisiones). 

- `launch/draw_and_obstacle.launch.py` — Igual que `draw_number_only` pero
  añade el nodo `obstacle_avoidance` que permite esquivar obstaculos.


Cómo ejecutar:

gitclone https://github.com/rbriz758/g12_prii3
cd g12_prii3/g12_prii3_ws
export TURTLEBOT3_MODEL=burger
colcon build
source install/setup.bash

------Lanzar solo el trazado en Gazebo

ros2 launch g12_prii3_move_turtlebot draw_number_only.launch.py


------Lanzar trazado + supervisor de colisión 

ros2 launch g12_prii3_move_turtlebot draw_and_collision.launch.py

//PARA PODER PROBAR LOS SENSORES HAY QUE COLOCAR UN CUBO U OBJETO Y COLOCARLO EN LA TRAYECTORIA

------Lanzar trazado + controlador reactivo

ros2 launch g12_prii3_move_turtlebot draw_and_obstacle.launch.py

//PARA PODER PROBAR LOS SENSORES HAY QUE COLOCAR UN CUBO U OBJETO DE TAMAÑO SIMILAR AL TURTLEBOT Y COLOCARLO EN LA TRAYECTORIA


Llamadas a servicios manuales (se ejecutan en otra terminal)

- Pausar el dibujo:


ros2 service call /detener_dibujo std_srvs/srv/Trigger


- Reanudar el dibujo:


ros2 service call /reanudar_dibujo std_srvs/srv/Trigger


- Reiniciar el dibujo:

ros2 service call /reiniciar_dibujo std_srvs/srv/Trigger


## g12_prii3_move_jetbot

ssh -X jetbot@[IP_del_robot]:
scp -r ~/Tercero/Proyecto/g12_prii3 g12_prii3_ws jetbot@<IP_DEL_ROBOT>:
Cómo ejecutar(todo desde la terminal del robot):

Terminal1:

ros2 launch jetbot_pro_ros2 jetbot.py

Terminal2:


colcon build
source install/setup.bash
ros2 launch g12_prii3_move_jetbot draw_number.launch.py


Parámetros de interés (declared en el nodo `DrawNumberJetBot`):

- `scale` — escala de la trayectoria.
- `linear_limit`, `angular_limit` — límites de velocidad.
- `path_separation` — separación entre trazos del número.

