				----SPRINT 3----INSTRUCCIONES----
				
IMPORTANTE!!!!

Si quieres utilizar los arucos, tienes que ir a la pestaña de insertar en gazebo y seleccionar uno de los 3 markets disponibles en funcion de tu ruta,ademas, el aruco se debe colocar en la columna de la de la puerta donde se queda esperando el robot a leer un market.



                                   -----COMANDOS REQUISITO----

export TURTLEBOT3_MODEL=waffle
source install/setup.bash
colcon build


                                --ABRIR PROGRAMAS POR SEPRADO----

Mundo gazebo F1L3:
ros2 launch g12_prii3_nav_turtlebot f1l3_world.launch.py

RVIZ:
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=src/g12_prii3_nav_turtlebot/maps/map.yaml

python3 (codigo a ejecutar)


                                         ---LAUNCH----


-LAUNCH que lanza F1L3 + Piloto automatico + nav2->

ros2 launch g12_prii3_nav_turtlebot project_launch.launch.py


-LAUNCH que lanza F1L3 + trayectoria predefinida->

ros2 launch g12_prii3_nav_turtlebot predefined_launch.launch.py
