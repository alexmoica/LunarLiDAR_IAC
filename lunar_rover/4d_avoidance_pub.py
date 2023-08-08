# Alex Moica 2023

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Point
from std_msgs.msg import Header, Bool
import math

class Avoidance4D(Node):
    def __init__(self):
        super().__init__('my_node')

        # Create a timer to call timer_callback every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)

        # Create a client to call the Gazebo GetEntityState service
        self.client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        
        # Store previous positions and timestamps for both rovers
        self.prev_positions = {
            'lunar_rover': {'position': None, 'timestamp': None},
            'dynamic_rover': {'position': None, 'timestamp': None}
        }
        
        self.prev_velocities = {
            'dynamic_rover': None
        }
        
        self.collision_publisher = self.create_publisher(Bool, 'collision_event', 10)
        self.no_collision_publisher = self.create_publisher(Point, 'no_collision_event', 10)

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
        collision_risk_flag = False
        try:
            response = future.result()
            position = response.state.pose.position
            timestamp = response.header.stamp.sec
        
            # Get previous position and timestamp of dynamic_rover
            prev_position_dynamic = self.prev_positions['dynamic_rover']['position']
            prev_timestamp_dynamic = self.prev_positions['dynamic_rover']['timestamp']
            prev_velocity_dynamic = self.prev_velocities['dynamic_rover']
            
            prev_position = self.prev_positions['lunar_rover']['position']
            prev_timestamp = self.prev_positions['lunar_rover']['timestamp']
            
            if prev_position is not None:
                # Calculate distance and time difference
                delta_x_distance = position.x - prev_position_dynamic.x
                delta_y_distance = position.y - prev_position_dynamic.y
                distance = math.sqrt(delta_x_distance ** 2 + delta_y_distance ** 2)
                delta_time = max(1, timestamp - prev_timestamp)
                
                # Calculate velocities
                delta_x = position.x - prev_position.x
                delta_y = position.y - prev_position.y
                velocity_x = delta_x / delta_time
                velocity_y = delta_y / delta_time
                
                # Check for collision risk based on distance and velocities
                if distance < 10.0:
                    # Calculate relative velocity
                    relative_velocity = (velocity_x - prev_velocity_dynamic['v_x'], velocity_y - prev_velocity_dynamic['v_y'])
                    
                    # Calculate dot product of relative_velocity and relative_position to find if objects are moving away or towards each other
                    dot_product = relative_velocity[0] * delta_x_distance + relative_velocity[1] * delta_y_distance
                    
                    if dot_product < 0 and 0 not in relative_velocity:
                        # Calculate time to collision
                        time_to_collision = distance / math.sqrt(relative_velocity[0] ** 2 + relative_velocity[1] ** 2)
                        if time_to_collision <= 4:
                            collision_risk_flag = True
                            self.get_logger().info(f'Collision risk detected! Time: {time_to_collision:.2f}, Distance: {distance:.2f}')
                            #self.collision_publisher.publish(Bool(data=True))
            if not collision_risk_flag:
                self.get_logger().info(f'No collision risk detected.')
                self.no_collision_publisher.publish(position)
            
            # Store current position and timestamp for lunar_rover
            self.prev_positions['lunar_rover']['position'] = position
            self.prev_positions['lunar_rover']['timestamp'] = timestamp
        except Exception as e:
            self.get_logger().info(f'Service call for lunar_rover failed: {e}')

    def service_callback_dynamic_rover(self, future):
        try:
            response = future.result()
            position = response.state.pose.position
            timestamp = response.header.stamp.sec
            
            # Get previous position and timestamp of dynamic_rover
            prev_position = self.prev_positions['dynamic_rover']['position']
            prev_timestamp = self.prev_positions['dynamic_rover']['timestamp']
            
            if prev_position is not None:
                # Calculate time difference and velocities
                delta_time = max(1, timestamp - prev_timestamp)
                delta_x = position.x - prev_position.x
                delta_y = position.y - prev_position.y
                velocity_x = delta_x / delta_time
                velocity_y = delta_y / delta_time
                self.prev_velocities['dynamic_rover'] = {'v_x': velocity_x, 'v_y': velocity_y}
            
            # Store current position and timestamp for dynamic_rover
            self.prev_positions['dynamic_rover']['position'] = position
            self.prev_positions['dynamic_rover']['timestamp'] = timestamp
        except Exception as e:
            self.get_logger().info(f'Service call for dynamic_rover failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Avoidance4D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

