#!/usr/bin/env python3

import math
from functools import partial
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(min(value, maximum), minimum)


class ObstacleAvoidanceController(Node):
    """Combina el trazado del número 12 con un giro de evasión reactivo."""

    def __init__(self) -> None:
        super().__init__('obstacle_avoidance_controller')

        # Parámetros ajustables para el comportamiento de evasión.
        self.declare_parameter('safe_distance', 0.4)
        self.declare_parameter('clear_distance', 0.55)
        self.declare_parameter('side_clearance', 0.3)
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('linear_speed_close', 0.06)
        self.declare_parameter('angular_speed', 0.9)
        self.declare_parameter('side_gain', 0.5)
        self.declare_parameter('clear_samples', 6)
        self.declare_parameter('forward_angle', 0.35)
        self.declare_parameter('turn_preference', 'left')

        self.safe_distance = float(self.get_parameter('safe_distance').value)
        self.clear_distance = float(self.get_parameter('clear_distance').value)
        self.side_clearance = float(self.get_parameter('side_clearance').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.linear_speed_close = float(self.get_parameter('linear_speed_close').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.side_gain = float(self.get_parameter('side_gain').value)
        self.clear_samples = max(1, int(self.get_parameter('clear_samples').value))
        self.forward_angle = float(self.get_parameter('forward_angle').value)
        preference = str(self.get_parameter('turn_preference').value).lower()
        self.turn_preference_left = preference != 'right'

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.detener_client = self.create_client(Trigger, 'detener_dibujo')
        self.reanudar_client = self.create_client(Trigger, 'reanudar_dibujo')

        for client, label in ((self.detener_client, 'detener_dibujo'),
                              (self.reanudar_client, 'reanudar_dibujo')):
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Esperando servicio {label}...')

        self.control_timer = self.create_timer(0.05, self.control_loop)

        # Distancias actuales calculadas a partir del LIDAR.
        self.front_distance: Optional[float] = None
        self.left_distance: Optional[float] = None
        self.right_distance: Optional[float] = None

        # Estado del algoritmo de evasión.
        self.avoiding = False
        self.avoid_direction = 1.0  # >0 gira izquierda, <0 gira derecha
        self.clear_counter = 0
        self.waiting_detener = False
        self.waiting_reanudar = False

        self.get_logger().info('Nodo obstacle_avoidance listo.')

    def scan_callback(self, msg: LaserScan) -> None:
        self.front_distance, self.left_distance, self.right_distance = self._compute_sectors(msg)

        if not self.avoiding:
            if self.front_distance is not None and self.front_distance < self.safe_distance:
                self.start_avoidance()
        else:
            if self._path_is_clear():
                self.clear_counter += 1
                if self.clear_counter >= self.clear_samples:
                    self.stop_avoidance()
            else:
                self.clear_counter = 0

    def control_loop(self) -> None:
        if not self.avoiding:
            return

        twist = Twist()

        forward = self.linear_speed
        if self.front_distance is not None and self.front_distance < self.safe_distance * 0.75:
            forward = self.linear_speed_close

        angular = self.avoid_direction * self.angular_speed
        angular += self._side_correction()

        twist.linear.x = forward
        twist.angular.z = clamp(angular, -1.8, 1.8)
        self.cmd_pub.publish(twist)

    def start_avoidance(self) -> None:
        if self.avoiding:
            return

        self.avoid_direction = self._choose_direction()
        self.get_logger().warn(
            f'Obstáculo frontal detectado. Maniobrando hacia {"izquierda" if self.avoid_direction > 0 else "derecha"}.'
        )
        self.avoiding = True
        self.clear_counter = 0
        self.waiting_reanudar = False

        if not self.waiting_detener:
            request = Trigger.Request()
            future = self.detener_client.call_async(request)
            future.add_done_callback(partial(self._handle_service_response, 'detener'))
            self.waiting_detener = True

    def stop_avoidance(self) -> None:
        if not self.avoiding:
            return

        self.get_logger().info('Trayectoria despejada. Reincorporando al dibujo del número.')
        self.avoiding = False
        self.clear_counter = 0
        self._publish_stop()

        if not self.waiting_reanudar:
            request = Trigger.Request()
            future = self.reanudar_client.call_async(request)
            future.add_done_callback(partial(self._handle_service_response, 'reanudar'))
            self.waiting_reanudar = True

    def _side_correction(self) -> float:
        correction = 0.0
        target = self.side_clearance

        if self.avoid_direction > 0:
            distance = self.right_distance
            if distance is not None:
                error = target - distance
                correction += self.side_gain * error
        else:
            distance = self.left_distance
            if distance is not None:
                error = distance - target
                correction += self.side_gain * error

        return clamp(correction, -1.0, 1.0)

    def _path_is_clear(self) -> bool:
        front_clear = self.front_distance is None or self.front_distance > self.clear_distance
        if self.avoid_direction > 0:
            side_clear = self.right_distance is None or self.right_distance > self.side_clearance
        else:
            side_clear = self.left_distance is None or self.left_distance > self.side_clearance
        return front_clear and side_clear

    def _choose_direction(self) -> float:
        left = self.left_distance if self.left_distance is not None else float('inf')
        right = self.right_distance if self.right_distance is not None else float('inf')

        if math.isinf(left) and math.isinf(right):
            return 1.0 if self.turn_preference_left else -1.0

        if left > right:
            return 1.0
        if right > left:
            return -1.0

        return 1.0 if self.turn_preference_left else -1.0

    def _compute_sectors(self, scan: LaserScan) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        front = []
        left = []
        right = []

        angle = scan.angle_min
        for distance in scan.ranges:
            if math.isinf(distance) or math.isnan(distance):
                angle += scan.angle_increment
                continue

            if -self.forward_angle <= angle <= self.forward_angle:
                front.append(distance)
            elif self.forward_angle < angle <= self.forward_angle + 0.7:
                left.append(distance)
            elif -self.forward_angle - 0.7 <= angle < -self.forward_angle:
                right.append(distance)

            angle += scan.angle_increment

        return (
            min(front) if front else None,
            min(left) if left else None,
            min(right) if right else None,
        )

    def _handle_service_response(self, action: str, future: Future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Error al solicitar {action}: {exc}')
            if action == 'detener':
                self.waiting_detener = False
            else:
                self.waiting_reanudar = False
            return

        if response.success:
            self.get_logger().info(f'Servicio {action} ejecutado: {response.message}')
        else:
            self.get_logger().warn(f'Servicio {action} no pudo completarse: {response.message}')

        if action == 'detener':
            self.waiting_detener = False
        else:
            self.waiting_reanudar = False

    def _publish_stop(self) -> None:
        twist = Twist()
        self.cmd_pub.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleAvoidanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo obstacle_avoidance interrumpido por el usuario.')
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
