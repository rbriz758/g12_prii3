#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger


class DrawNumberGazebo(Node):
    def __init__(self) -> None:
        super().__init__('draw_number_gazebo')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.create_service(Trigger, 'detener_dibujo', self.detener_callback)
        self.create_service(Trigger, 'reanudar_dibujo', self.reanudar_callback)
        self.create_service(Trigger, 'reiniciar_dibujo', self.reiniciar_callback)

        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.current_pose: Optional[Tuple[float, float, float]] = None
        self.initial_pose: Optional[Tuple[float, float, float]] = None
        self.targets_world: Optional[List[Tuple[float, float]]] = None
        self.segment_idx = 0
        self.paused = False
        self.finished = False

        # Gains and limits tuned for TurtleBot3 in Gazebo
        self.k_linear = 1.1
        self.k_angular = 3.0
        self.max_linear = 0.22
        self.max_linear_close = 0.12
        self.min_linear = 0.06
        self.max_angular = 2.2
        self.distance_tolerance = 0.03
        self.heading_threshold = 0.35

        # Points relative to the starting pose that outline the number "12"
        scale = 1.6  # Escala global para agrandar el número en Gazebo
        base_path: List[Tuple[float, float]] = [
            (0.0, 1.0),   # Trazo vertical del "1"
            (0.4, 1.0),   # Transición hacia el "2"
            (1.0, 1.0),   # Segmento superior del "2"
            (1.0, 0.55),  # Inicio de la diagonal descendente
            (0.35, 0.15), # Base izquierda del "2"
            (1.0, 0.15),  # Segmento inferior del "2"
        ]
        self.path_rel: List[Tuple[float, float]] = [
            (x * scale, y * scale) for (x, y) in base_path
        ]

        self.get_logger().info(
            'Nodo listo. Esperando odometría para iniciar el recorrido del número 12.'
        )

    def odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = self.yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)

        self.current_pose = (position.x, position.y, yaw)

        if self.initial_pose is None and not self.paused:
            self.initial_pose = self.current_pose
            self.targets_world = [self.transform_to_world(point) for point in self.path_rel]
            self.segment_idx = 0
            self.finished = False
            self.get_logger().info('Origen fijado. Comenzando a trazar el número 12.')

    def control_loop(self) -> None:
        if self.paused or self.finished:
            self.publish_stop()
            return

        if self.current_pose is None or self.initial_pose is None or self.targets_world is None:
            return

        if self.segment_idx >= len(self.targets_world):
            self.finish_path()
            return

        target = self.targets_world[self.segment_idx]
        current_x, current_y, current_yaw = self.current_pose

        dx = target[0] - current_x
        dy = target[1] - current_y
        distance = math.hypot(dx, dy)

        if distance <= self.distance_tolerance:
            self.segment_idx += 1
            if self.segment_idx >= len(self.targets_world):
                self.finish_path()
            return

        desired_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(desired_heading - current_yaw)

        angular_speed = max(min(self.k_angular * heading_error, self.max_angular), -self.max_angular)

        linear_speed = 0.0
        if abs(heading_error) < self.heading_threshold:
            raw_linear = self.k_linear * distance
            if distance < 0.15:
                raw_linear = min(raw_linear, self.max_linear_close)
            else:
                raw_linear = min(raw_linear, self.max_linear)

            if distance > self.distance_tolerance * 2:
                linear_speed = max(raw_linear, self.min_linear)
            else:
                linear_speed = min(raw_linear, self.max_linear_close)

        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.cmd_pub.publish(twist)

    def detener_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.paused = True
        self.publish_stop()
        response.success = True
        response.message = 'Dibujo pausado.'
        return response

    def reanudar_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.finished:
            response.success = False
            response.message = 'El dibujo ya se completó. Usa reiniciar_dibujo para repetirlo.'
            return response

        self.paused = False
        response.success = True
        response.message = 'Dibujo reanudado.'
        return response

    def reiniciar_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.paused = False
        self.finished = False
        self.segment_idx = 0
        self.initial_pose = None
        self.targets_world = None
        self.publish_stop()
        response.success = True
        response.message = 'Estado reiniciado. Esperando nueva odometría.'
        return response

    def finish_path(self) -> None:
        self.finished = True
        self.publish_stop()
        self.get_logger().info('Número 12 completado correctamente.')

    def publish_stop(self) -> None:
        twist = Twist()
        self.cmd_pub.publish(twist)

    def transform_to_world(self, point: Tuple[float, float]) -> Tuple[float, float]:
        if self.initial_pose is None:
            return point

        origin_x, origin_y, origin_yaw = self.initial_pose
        rel_x, rel_y = point

        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)

        world_x = origin_x + rel_x * cos_yaw - rel_y * sin_yaw
        world_y = origin_y + rel_x * sin_yaw + rel_y * cos_yaw
        return (world_x, world_y)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = DrawNumberGazebo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo interrumpido por el usuario.')
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()