# Alex Moica 2023

import rclpy
import time
import re
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from lunar_rover.action import Avoidance


class AvoidanceActionServer(Node):

    def __init__(self):
        super().__init__('avoidance_action_server')
        self._action_server = ActionServer(
            self,
            Avoidance,
            'avoidance',
            self.execute_callback)
        self._goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.z = 0.239989
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        if goal_handle.request.command == "avoid":
            avoidance_goal = PoseStamped()
            avoidance_goal.header.frame_id = "map"
            avoidance_goal.pose.position.x = -4.0
            avoidance_goal.pose.position.y = 4.0
            avoidance_goal.pose.position.z = 0.0
            avoidance_goal.pose.orientation.x = 0.0
            avoidance_goal.pose.orientation.y = 0.0
            avoidance_goal.pose.orientation.z = 0.0
            avoidance_goal.pose.orientation.w = 1.0

            avoid_length = 6

            # Publish the initial goal pose
            self._goal_publisher.publish(goal)
            self.get_logger().info('Published initial goal.')

            try:
                # Sleep for 5 seconds
                time.sleep(5)

                # Publish the avoidance goal pose
                self._goal_publisher.publish(avoidance_goal)
                self.get_logger().info('Published avoidance goal.')

                # Sleep for the specified avoidance length
                time.sleep(avoid_length)

                # Re-publish the initial goal pose
                self._goal_publisher.publish(goal)
                self.get_logger().info('Re-published initial goal.')

            except Exception as e:
                self.get_logger().error(f"Error occurred: {e}")
        elif "start" in goal_handle.request.command:
            match = re.match(r"start \(([-0-9.]+), ([-0-9.]+)\)", goal_handle.request.command)
            if match:
                x_value = float(match.group(1))
                y_value = float(match.group(2))
                goal.pose.position.x = x_value + 0.8
                goal.pose.position.y = y_value - 0.8
                
                if goal.pose.position.x < 13.5:
                    self._goal_publisher.publish(goal)
                    self.get_logger().info('Published initial goal.')
                else:
                    self.get_logger().info('End of path.')
        else:
            self.get_logger().info('Ignoring unknown command.')
            
        # Create an instance of the result message
        result = Avoidance.Result()

        # Populate the result field with the desired value
        result.result = "Success" 

        # Return the result instance as the action server's response
        return result


def main(args=None):
    rclpy.init(args=args)

    avoidance_action_server = AvoidanceActionServer()

    rclpy.spin(avoidance_action_server)


if __name__ == '__main__':
    main()

