# Alex Moica 2023

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import math

class Avoidance3D(Node):
    def __init__(self):
        super().__init__('my_node')

        # Create a timer to call timer_callback every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)

        # Create a client to call the Gazebo GetEntityState service
        self.client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        
        # Store previous positions for both rovers
        self.prev_positions = {
            'lunar_rover': None,
            'dynamic_rover': None
        }

    def timer_callback(self):
        # Wait for the service to be available
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available')
            return
        
        # Request and handle responses for both rovers
        request_dynamic_rover = GetEntityState.Request()
        request_dynamic_rover.name = 'dynamic_rover'
        request_dynamic_rover.reference_frame = 'world'
        
        request_lunar_rover = GetEntityState.Request()
        request_lunar_rover.name = 'lunar_rover'
        request_lunar_rover.reference_frame = 'world'
        
        # Call the service asynchronously for dynamic_rover
        future_dynamic_rover = self.client.call_async(request_dynamic_rover)
        future_dynamic_rover.add_done_callback(self.service_callback_dynamic_rover)
        
        # Call the service asynchronously for lunar_rover
        future_lunar_rover = self.client.call_async(request_lunar_rover)
        future_lunar_rover.add_done_callback(self.service_callback_lunar_rover)

    def service_callback_lunar_rover(self, future):
        try:
            response = future.result()
            position = response.state.pose.position
        
            # Get previous position of dynamic_rover
            prev_position_dynamic = self.prev_positions['dynamic_rover']
            
            if prev_position_dynamic is not None:
                delta_x = position.x - prev_position_dynamic.x
                delta_y = position.y - prev_position_dynamic.y
                distance = math.sqrt(delta_x ** 2 + delta_y ** 2)
                
                # Check for collision risk based on distance
                if distance < 4.0:
                    self.get_logger().info(f'Collision risk detected! Distance: {distance:.2f}')
            
            # Store current position for lunar_rover
            self.prev_positions['lunar_rover'] = position
        except Exception as e:
            self.get_logger().info(f'Service call for lunar_rover failed: {e}')

    def service_callback_dynamic_rover(self, future):
        try:
            response = future.result()
            position = response.state.pose.position
            
            # Get previous position of dynamic_rover
            prev_position = self.prev_positions['dynamic_rover']
            
            # Store current position for dynamic_rover
            self.prev_positions['dynamic_rover'] = position
        except Exception as e:
            self.get_logger().info(f'Service call for dynamic_rover failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Avoidance3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

