import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rodrigo/Proyecto_III/g12_prii3/install/g12_prii3_move_turtlebot'
