# Alex Moica 2023

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from lunar_rover.action import Avoidance
from rclpy.action import ActionClient


class AvoidanceSubscriber(Node):

    def __init__(self):
        super().__init__('avoidance_subscriber')
        self.collision_subscription = self.create_subscription(
            Bool,
            'collision_event',
            self.collision_listener_callback,
            10)
        self.no_collision_subscription = self.create_subscription(
            Point,
            'no_collision_event',
            self.no_collision_listener_callback,
            10)
        self.avoidance_client = ActionClient(self, Avoidance, 'avoidance')

    def collision_listener_callback(self, msg):
        if msg.data:
            self.get_logger().info('Collision risk detected.')
            # run avoidance

    def no_collision_listener_callback(self, msg):
        if msg.x:
            self.get_logger().info('No collision risk detected.')
            self.send_start_command(msg)

    def send_start_command(self, point_msg):
        if not self.avoidance_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Avoidance action server not available, waiting...')
            return

        goal_msg = Avoidance.Goal()
        goal_msg.command = f"start ({point_msg.x}, {point_msg.y})"

        goal_handle = self.avoidance_client.send_goal_async(goal_msg)
        goal_handle.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('Start command sent to action server.')
            else:
                self.get_logger().info('Failed to send start command to action server.')
        except Exception as e:
            self.get_logger().error(f"Error occurred: {e}")

def main(args=None):
    rclpy.init(args=args)

    avoidance_subscriber = AvoidanceSubscriber()

    rclpy.spin(avoidance_subscriber)

    avoidance_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

