import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from example_interfaces.srv import Trigger


class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.move_callback)
        self.active = True
        self.step = 0

        # Servicio pausar/reanudar
        self.srv_toggle = self.create_service(SetBool, 'toggle_motion', self.toggle_cb)
        # Servicio reiniciar
        self.srv_reset = self.create_service(Trigger, 'reset_motion', self.reset_cb)

    def toggle_cb(self, request, response):
        self.active = request.data
        response.success = True
        response.message = f'Motion {"enabled" if self.active else "paused"}'
        return response

    def reset_cb(self, request, response):
        self.step = 0
        response.success = True
        response.message = 'Motion reset'
        return response

    def move_callback(self):
        if not self.active:
            return
        msg = Twist()

        # === Dibujar número 1 ===
        if self.step < 10:
            msg.linear.x = 2.0  # Línea vertical
        elif self.step < 12:
            msg.linear.x = 0.0  # Pausa

        # === Dibujar número 2 ===
        elif self.step < 22:
            msg.angular.z = 1.5   # Curva superior
        elif self.step < 32:
            msg.linear.x = 2.0    # Línea diagonal
        elif self.step < 42:
            msg.angular.z = -1.5  # Curva inferior
        else:
            self.step = 0         # Reinicio

        self.step += 1
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
