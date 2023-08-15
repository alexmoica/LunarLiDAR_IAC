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
        
        avoidance_goal = PoseStamped()
        avoidance_goal.header.frame_id = "map"
        avoidance_goal.pose.position.z = 0.239989
        avoidance_goal.pose.orientation.x = 0.0
        avoidance_goal.pose.orientation.y = 0.0
        avoidance_goal.pose.orientation.z = 0.0
        avoidance_goal.pose.orientation.w = 1.0

        if "avoid" in goal_handle.request.command:
            match = re.match(r"avoid \(([-0-9.]+), ([-0-9.]+), ([-0-9.]+), ([-0-9.]+)\)", goal_handle.request.command)
            if match:
                x_value = float(match.group(1))
                y_value = float(match.group(2))
                x_swerve = float(match.group(3))
                y_swerve = float(match.group(4))
            
                avoidance_goal.pose.position.x = x_value + x_swerve
                avoidance_goal.pose.position.y = y_value + y_swerve

                if avoidance_goal.pose.position.y <= -9 and avoidance_goal.pose.position.x >= 9:
                    self.get_logger().info('End of path.')
                else:
                    # Publish the avoidance goal pose
                    self._goal_publisher.publish(avoidance_goal)
                    self.get_logger().info('Published avoidance goal.')
                
        elif "start" in goal_handle.request.command:
            match = re.match(r"start \(([-0-9.]+), ([-0-9.]+)\)", goal_handle.request.command)
            if match:
                x_value = float(match.group(1))
                y_value = float(match.group(2))
                if x_value >= 9:
                    goal.pose.position.x = x_value
                else:
                    goal.pose.position.x = max(min(x_value + 0.8, 14.5), -0.5) # bound between -0.5 and 14.5
                
                if y_value <= -9:
                    goal.pose.position.y = y_value
                else:
                    goal.pose.position.y = min(max(y_value - 0.8, -14.5), 0.5) # bound between 0.5 and -14.5
               
                self._goal_publisher.publish(goal)
                self.get_logger().info('Published initial goal.')
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

