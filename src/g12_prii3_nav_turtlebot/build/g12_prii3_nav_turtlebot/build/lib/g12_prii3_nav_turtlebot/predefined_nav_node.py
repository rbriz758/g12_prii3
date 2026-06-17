#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class PredefinedNavNode(Node):
    def __init__(self):
        super().__init__('predefined_nav_node')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        try:
            self.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass
        self.get_logger().info('Predefined navigation node started')
        
        # Wait for server once at startup
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        self.send_goal()

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Target pose
        goal_msg.pose.pose.position.x = 6.06
        goal_msg.pose.pose.position.y = 15.19
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info('Sending goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected. Waiting for Initial Pose... Retrying in 3 seconds.')
            self._retry_timer = self.create_timer(3.0, self.retry_goal)
            return

        self.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def retry_goal(self):
        if hasattr(self, '_retry_timer'):
            self._retry_timer.cancel()
        self.send_goal()

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: Goal reached successfully!')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PredefinedNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    finally:
        # Check if shutdown was already called
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
