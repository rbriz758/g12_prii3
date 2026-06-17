import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
from turtlesim.msg import Pose
from std_srvs.srv import Trigger, Empty

class MoverTortuga(Node):
    def __init__(self):
        super().__init__('mover_tortuga')

        # --- Creación de la tortuga (nodo principal) ---
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)

       
        self.teleport_cli = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        self.setpen_cli = self.create_client(SetPen, 'turtle1/set_pen')
        self.clear_cli = self.create_client(Empty, 'clear')

        while not self.teleport_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio teleport_absolute...')
        while not self.setpen_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio set_pen...')
        while not self.clear_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio clear...')

        # --- Declaración de los servicios de parar / reanudar / reiniciar ---
        self.detener_srv = self.create_service(Trigger, 'detener_dibujo', self.detener_callback)
        self.reanudar_srv = self.create_service(Trigger, 'reanudar_dibujo', self.reanudar_callback)
        self.reiniciar_srv = self.create_service(Trigger, 'reiniciar_dibujo', self.reiniciar_callback)

        
        self.pose = None
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state = 'levantar_lapiz_inicio'
        self.segmento = 0
        self.pausado = False
        self.estado_guardado = None

    def pose_callback(self, msg):
        self.pose = msg

    # --- Aquí servicio para detener ---
    def detener_callback(self, request, response):
        self.pausado = True
        self.estado_guardado = self.state
        response.success = True
        response.message = 'Dibujo detenido.'
        return response

    # --- Aquí servicio reanudar ---
    def reanudar_callback(self, request, response):
        if self.estado_guardado:
            self.state = self.estado_guardado
        self.pausado = False
        response.success = True
        response.message = 'Dibujo reanudado.'
        return response

    # --- Aquí servicio reiniciar ---
    def reiniciar_callback(self, request, response):
        self.pausado = False
        self.state = 'levantar_lapiz_inicio'
        self.segmento = 0

        # Limpiar pantalla
        clear_req = Empty.Request()
        self.clear_cli.call_async(clear_req)

        # Volver a la posición inicial
        tp_req = TeleportAbsolute.Request()
        tp_req.x = 3.5
        tp_req.y = 8.0
        tp_req.theta = 0.0
        self.teleport_cli.call_async(tp_req)

        # Levantar lápiz
        pen_req = SetPen.Request()
        pen_req.off = True
        self.setpen_cli.call_async(pen_req)

        response.success = True
        response.message = 'Dibujo reiniciado y pantalla limpiada.'
        return response

    def timer_callback(self):
        if self.pose is None or self.pausado:
            return

        twist = Twist()

        # ============================================================
        # === Aquí es donde se dibuja el número 1 ===
        
        if self.state == 'levantar_lapiz_inicio':
            pen_req = SetPen.Request()
            pen_req.off = True
            self.setpen_cli.call_async(pen_req)
            self.state = 'teleport_inicio_1'

        elif self.state == 'teleport_inicio_1':
            tp_req = TeleportAbsolute.Request()
            tp_req.x = 3.5
            tp_req.y = 8.0
            tp_req.theta = 0.0
            self.teleport_cli.call_async(tp_req)
            self.state = 'bajar_lapiz_1'

        elif self.state == 'bajar_lapiz_1':
            pen_req = SetPen.Request()
            pen_req.off = False
            pen_req.r = 255
            pen_req.g = 255
            pen_req.b = 255
            pen_req.width = 3
            self.setpen_cli.call_async(pen_req)
            self.state = 'dibujar_1'

        elif self.state == 'dibujar_1':
            if self.pose.y > 4.0:
                twist.linear.y = -2.0
            else:
                twist.linear.y = 0.0
                self.state = 'levantar_lapiz'

        # ============================================================
        # === Aquí se teleporta del número 1 al número 2 ===
        
        elif self.state == 'levantar_lapiz':
            pen_req = SetPen.Request()
            pen_req.off = True
            self.setpen_cli.call_async(pen_req)
            self.state = 'teleport_derecha'

        elif self.state == 'teleport_derecha':
            tp_req = TeleportAbsolute.Request()
            tp_req.x = 6.0
            tp_req.y = 8.0
            tp_req.theta = 0.0
            self.teleport_cli.call_async(tp_req)
            self.state = 'bajar_lapiz_2'

        # ============================================================
        # === Aquí se dibuja el número 2 ===
        
        elif self.state == 'bajar_lapiz_2':
            pen_req = SetPen.Request()
            pen_req.off = False
            pen_req.r = 255
            pen_req.g = 255
            pen_req.b = 255
            pen_req.width = 3
            self.setpen_cli.call_async(pen_req)
            self.state = 'dibujar_2'
            self.segmento = 0

        elif self.state == 'dibujar_2':
            if self.segmento == 0:
                if self.pose.x < 8.5:
                    twist.linear.x = 2.0
                else:
                    twist.linear.x = 0.0
                    self.segmento += 1
            elif self.segmento == 1:
                if self.pose.y > 6.0:
                    twist.linear.y = -2.0
                else:
                    twist.linear.y = 0.0
                    self.segmento += 1
            elif self.segmento == 2:
                if self.pose.x > 6.0:
                    twist.linear.x = -2.0
                else:
                    twist.linear.x = 0.0
                    self.segmento += 1
            elif self.segmento == 3:
                if self.pose.y > 4.0:
                    twist.linear.y = -2.0
                else:
                    twist.linear.y = 0.0
                    self.segmento += 1
            elif self.segmento == 4:
                if self.pose.x < 8.5:
                    twist.linear.x = 2.0
                else:
                    twist.linear.x = 0.0
                    self.state = 'finalizado'
                    # Mensaje de confirmacion que el numero 1 y 2 se han dibujado
                    self.get_logger().info('Número 1 y 2 dibujados correctamente')

        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    nodo = MoverTortuga()  # Creación de la tortuga
    rclpy.spin(nodo)       
    nodo.destroy_node()    
    rclpy.shutdown()       # Cierra ROS 2

if __name__ == '__main__':
    main()
