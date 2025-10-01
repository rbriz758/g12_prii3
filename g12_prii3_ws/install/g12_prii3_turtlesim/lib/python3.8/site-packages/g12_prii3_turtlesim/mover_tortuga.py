import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
from turtlesim.msg import Pose

class MoverTortuga(Node):
    def __init__(self):
        super().__init__('mover_tortuga')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)

        # Servicios
        self.teleport_cli = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        self.setpen_cli = self.create_client(SetPen, 'turtle1/set_pen')
        while not self.teleport_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio teleport_absolute...')
        while not self.setpen_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio set_pen...')

        self.pose = None
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state = 'levantar_lapiz_inicio'
        self.segmento = 0

    def pose_callback(self, msg):
        self.pose = msg

    def timer_callback(self):
        if self.pose is None:
            return

        twist = Twist()

        if self.state == 'levantar_lapiz_inicio':
            pen_req = SetPen.Request()
            pen_req.off = True  # Levanta el lápiz
            self.setpen_cli.call_async(pen_req)
            self.state = 'teleport_inicio_1'

        elif self.state == 'teleport_inicio_1':
            tp_req = TeleportAbsolute.Request()
            tp_req.x = 3.5  # Posición ajustada del "1"
            tp_req.y = 8.0
            tp_req.theta = 0.0
            self.teleport_cli.call_async(tp_req)
            self.state = 'bajar_lapiz_1'

        elif self.state == 'bajar_lapiz_1':
            pen_req = SetPen.Request()
            pen_req.off = False  # Baja el lápiz
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
                    self.get_logger().info('Número 1 y 2 dibujados correctamente')

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    nodo = MoverTortuga()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()