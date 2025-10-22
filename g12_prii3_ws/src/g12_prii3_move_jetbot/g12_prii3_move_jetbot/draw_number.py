#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class JetBotSegmento(Node):
    def __init__(self):
        super().__init__('jetbot_segmento_dos')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz
        self.get_logger().info("Nodo JetBotSegmento iniciado correctamente")

    def mover(self, linear=0.0, angular=0.0, duracion=1.0):
        """Mueve el robot durante 'duracion' segundos con las velocidades dadas"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        pasos = int(duracion * 10)  # a 10 Hz
        for _ in range(pasos):
            self.publisher_.publish(msg)
            self.rate.sleep()

        self.detener()

    def detener(self):
        """Detiene el robot"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        for _ in range(5):
            self.publisher_.publish(msg)
            self.rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    robot = JetBotSegmento()

    # === Segmento A (arriba) ===
    robot.mover(linear=0.2, duracion=2.0)

    # === Girar derecha (hacia segmento B) ===
    robot.mover(angular=-0.6, duracion=1.1)

    # === Segmento B (vertical hacia abajo) ===
    robot.mover(linear=0.2, duracion=2.0)

    # === Girar izquierda (hacia segmento G) ===
    robot.mover(angular=0.6, duracion=1.1)

    # === Segmento G (horizontal en medio, hacia izquierda) ===
    robot.mover(linear=0.2, duracion=2.0)

    # === Girar derecha (hacia segmento E) ===
    robot.mover(angular=-0.6, duracion=1.1)

    # === Segmento E (vertical hacia abajo) ===
    robot.mover(linear=0.2, duracion=2.0)

    # === Girar izquierda (hacia segmento D) ===
    robot.mover(angular=0.6, duracion=1.1)

    # === Segmento D (horizontal abajo) ===
    robot.mover(linear=0.2, duracion=2.0)

    robot.detener()
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

