import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav2_msgs.action import NavigateToPose
import cv2
import cv2.aruco
import numpy as np
import time

# --- CLASE NAVEGADOR (NO TOCAR) ---
class SimpleNavigator:
    def __init__(self, node):
        self.node = node
        self.client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.goal_handle = None
        self.result_future = None
        
    def waitUntilNav2Active(self):
        self.node.get_logger().info('Esperando a Nav2...')
        self.client.wait_for_server()
        self.node.get_logger().info('Nav2 activo.')

    def goToPose(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.node.get_logger().info('Enviando robot a nueva posición...')
        self.goal_handle = None
        self.result_future = None
        
        send_goal_future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.node.get_logger().error('La meta fue rechazada por Nav2')
            return False
        
        self.result_future = self.goal_handle.get_result_async()
        return True

    def isTaskComplete(self):
        if self.result_future and self.result_future.done():
            return True
        rclpy.spin_once(self.node, timeout_sec=0.1)
        return False

    def getResult(self):
        if self.result_future:
            status = self.result_future.result().status
            if status == 4: return True
        return False

# --- DETECTOR ARUCO ---
class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.last_id = None
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.listener_callback,
            10)
            
        # Probamos varios diccionarios por seguridad
        self.diccionarios_a_probar = [
            cv2.aruco.DICT_4X4_50,
            cv2.aruco.DICT_5X5_100,   # Probablemente el tuyo sea este
            cv2.aruco.DICT_ARUCO_ORIGINAL,
            cv2.aruco.DICT_6X6_250
        ]
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def listener_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            image_np = np_arr.reshape((msg.height, msg.width, -1))

            if 'rgb' in msg.encoding:
                cv_image = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)
            else:
                cv_image = image_np

            for dic_id in self.diccionarios_a_probar:
                aruco_dict = cv2.aruco.Dictionary_get(dic_id)
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    cv_image, aruco_dict, parameters=self.aruco_params
                )
                
                if ids is not None and len(ids) > 0:
                    self.last_id = ids[0][0]
                    return

        except Exception as e:
            pass

def create_pose(node, x, y, w=1.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = float(w)
    pose.pose.orientation.z = 0.0
    return pose

# --- MAIN ---
def main():
    rclpy.init()
    aruco_node = ArucoDetector()
    nav = SimpleNavigator(aruco_node) 

    # =========================================================
    # TUS COORDENADAS REALES (Actualizadas)
    # =========================================================
    
    # Puerta (Donde lee)
    COORD_PUERTA = {'x': 12.467, 'y': 1.634, 'w': 1.0} 
    
    # Posiciones finales
    POSICION_1 = {'x': 6.329,  'y': 7.678,   'w': 1.0} 
    POSICION_2 = {'x': 13.155, 'y': 15.256,  'w': 1.0} 
    POSICION_3 = {'x': 6.375,  'y': 15.157,  'w': 1.0} 
    
    # Puntos intermedios para Ruta 2 y 3
    INTERMEDIO_3 = {'x': 12.111, 'y': 15.198, 'w': 1.0}
    # =========================================================

    nav.waitUntilNav2Active()

    # 1. Ir a la puerta
    print(f"Yendo a la puerta ({COORD_PUERTA['x']}, {COORD_PUERTA['y']})...")
    pose_lectura = create_pose(aruco_node, COORD_PUERTA['x'], COORD_PUERTA['y'], COORD_PUERTA['w'])
    
    # Bucle de reintento por si Nav2 rechaza la meta (ej. falta Initial Pose)
    while not nav.goToPose(pose_lectura):
        print("Meta rechazada. ¿Has dado la posición inicial en RViz? Reintentando en 3s...")
        time.sleep(3.0)

    while not nav.isTaskComplete():
        pass 

    # 2. Esperar a ver ArUco
    print("Llegué. Buscando código...")
    id_final = None
    
    contador_debug = 0
    while id_final is None:
        rclpy.spin_once(aruco_node, timeout_sec=0.1)
        
        if aruco_node.last_id is not None:
            id_final = aruco_node.last_id
            print(f"\n¡DETECTADO ID: {id_final}!")
            break
        
        contador_debug += 1
        if contador_debug % 20 == 0: print(".", end="", flush=True)

    print("") 

    # 3. Seleccionar destino según TUS IDs
    ruta_waypoints = []
    
    if id_final == 5: 
        ruta_waypoints = [POSICION_1]
        print("ID 5 -> Destino: POSICION 1")
        
    elif id_final == 17: 
        ruta_waypoints = [INTERMEDIO_3, POSICION_2]
        print("ID 17 -> Destino: POSICION 2 (con intermediario)")
        
    elif id_final == 6: 
        ruta_waypoints = [INTERMEDIO_3, POSICION_3]
        print("ID 6 -> Destino: POSICION 3 (con intermediario)")
        
    else:
        print(f"ID {id_final} no está en la lista (5, 17, 6). Me quedo aquí.")
        rclpy.shutdown()
        return

    # 4. Navegar
    print(f"Navegando ruta con {len(ruta_waypoints)} puntos...")
    for i, pt in enumerate(ruta_waypoints):
        print(f"Yendo al punto {i+1}/{len(ruta_waypoints)}: ({pt['x']}, {pt['y']})...")
        pose_pt = create_pose(aruco_node, pt['x'], pt['y'], pt['w'])
        nav.goToPose(pose_pt)
        while not nav.isTaskComplete():
            pass

    print("Misión cumplida.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()