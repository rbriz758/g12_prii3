import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rodrigo/Proyecto_III/g12_prii3/g12_prii3_ws/src/g12_prii3_nav_turtlebot/install/g12_prii3_nav_turtlebot'
