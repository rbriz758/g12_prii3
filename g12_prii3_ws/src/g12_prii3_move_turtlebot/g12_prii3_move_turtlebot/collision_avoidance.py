#!/usr/bin/env python3

import math
from functools import partial
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger


class CollisionAvoidanceSupervisor(Node):
    def __init__(self) -> None:
        super().__init__('collision_avoidance_supervisor')

        self.declare_parameter('safe_distance', 0.35)
        self.declare_parameter('forward_angle', 0.35)
        self.declare_parameter('stop_publish_rate', 10.0)
        self.declare_parameter('detener_service', 'detener_dibujo')
        self.declare_parameter('reanudar_service', 'reanudar_dibujo')

        self.safe_distance = float(self.get_parameter('safe_distance').value)
        self.forward_angle = float(self.get_parameter('forward_angle').value)

        stop_rate = float(self.get_parameter('stop_publish_rate').value)
        stop_period = 1.0 / stop_rate if stop_rate > 0 else 0.1

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        detener_name = str(self.get_parameter('detener_service').value)
        reanudar_name = str(self.get_parameter('reanudar_service').value)

        self.detener_client = self.create_client(Trigger, detener_name)
        self.reanudar_client = self.create_client(Trigger, reanudar_name)

        self.stop_timer = self.create_timer(stop_period, self.publish_stop_if_needed)

        self.obstacle_detected = False
        self.paused_by_collision = False
        self.closest_obstacle: Optional[float] = None

        for client, label in ((self.detener_client, detener_name), (self.reanudar_client, reanudar_name)):
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Esperando servicio {label}...')

        self.get_logger().info('Supervisor de evitación de obstáculos listo.')

    def scan_callback(self, msg: LaserScan) -> None:
        min_distance = self.minimum_distance(msg)
        previous_state = self.obstacle_detected

        self.closest_obstacle = min_distance
        if min_distance is not None and min_distance < self.safe_distance:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

        if self.obstacle_detected and not previous_state:
            if min_distance is not None:
                self.get_logger().warn(
                    f'Obstáculo detectado a {min_distance:.2f} m. Solicitando parada del dibujo.'
                )
            else:
                self.get_logger().warn('Obstáculo detectado. Solicitando parada del dibujo.')
            self.pause_draw()
        elif not self.obstacle_detected and previous_state:
            self.get_logger().info('Trayectoria despejada. Solicitando reanudación del dibujo.')
            self.resume_draw()

    def pause_draw(self) -> None:
        if self.paused_by_collision:
            return

        request = Trigger.Request()
        future = self.detener_client.call_async(request)
        future.add_done_callback(partial(self.handle_service_response, 'detener'))
        self.paused_by_collision = True
        self.publish_stop()

    def resume_draw(self) -> None:
        if not self.paused_by_collision:
            return

        request = Trigger.Request()
        future = self.reanudar_client.call_async(request)
        future.add_done_callback(partial(self.handle_service_response, 'reanudar'))
        self.paused_by_collision = False

    def publish_stop_if_needed(self) -> None:
        if self.obstacle_detected:
            self.publish_stop()

    def publish_stop(self) -> None:
        twist = Twist()
        self.cmd_pub.publish(twist)

    def handle_service_response(self, action: str, future: Future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Error al solicitar {action}: {exc}')
            if action == 'reanudar':
                self.paused_by_collision = True
            return

        if response.success:
            self.get_logger().info(f'Servicio {action} ejecutado: {response.message}')
        else:
            self.get_logger().warn(f'Servicio {action} no pudo completarse: {response.message}')
            if action == 'reanudar':
                self.paused_by_collision = True

    def minimum_distance(self, scan: LaserScan) -> Optional[float]:
        relevant = []
        angle = scan.angle_min
        for distance in scan.ranges:
            if abs(angle) <= self.forward_angle and not math.isinf(distance) and not math.isnan(distance):
                relevant.append(distance)
            angle += scan.angle_increment

        return min(relevant) if relevant else None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CollisionAvoidanceSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Supervisor interrumpido por el usuario.')
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
