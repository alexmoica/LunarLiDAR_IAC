# Alex Moica 2023

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Point
from std_msgs.msg import Header, Float32MultiArray
from datetime import datetime
import math
import csv
import os
import time

class Avoidance3D(Node):
    def __init__(self):
        super().__init__('Avoidance3D')

        # Create a timer to call timer_callback every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)

        # Create a client to call the Gazebo GetEntityState service
        self.client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        
        # Store previous positions for both rovers
        self.prev_positions = {
            'lunar_rover': None,
            'dynamic_rover': None
        }
        
        self.collision_publisher = self.create_publisher(Float32MultiArray, 'collision_event', 10)
        self.no_collision_publisher = self.create_publisher(Point, 'no_collision_event', 10)
        self.collision_flag = False
        self.result_filename = "result_log.csv"
        self.flag_finished = False
        self.flag_initial = True
        self.start_time = 0

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
            
            prev_position = self.prev_positions['lunar_rover']
            
            if position.y <= -9 and position.x >= 9 and not self.flag_finished:
                if not os.path.exists(self.result_filename):
                    with open(self.result_filename, 'w', newline='') as file:
                        writer = csv.writer(file)
                        current_datetime = datetime.now()
                        elapsed_time = time.time() - self.start_time
                        writer.writerow(["Time of Test", "3D/4D", "Collision", "Time Taken to Goal (s)"])
                        writer.writerow([current_datetime.strftime("%Y-%m-%d %H:%M:%S"), "3D", self.collision_flag, elapsed_time])
                else:
                    with open(self.result_filename, 'a', newline='') as file: 
                        writer = csv.writer(file)
                        current_datetime = datetime.now()
                        elapsed_time = time.time() - self.start_time
                        writer.writerow([current_datetime.strftime("%Y-%m-%d %H:%M:%S"), "3D", self.collision_flag, elapsed_time])
                self.flag_finished = True
                self.get_logger().info(f'End of path.')
            
            if prev_position is not None and not self.flag_finished:
                if self.flag_initial:
                    self.flag_initial = False
                    self.start_time = time.time()

                # Calculate distance
                delta_x_distance = position.x - prev_position_dynamic.x
                delta_y_distance = position.y - prev_position_dynamic.y
                distance = math.sqrt(delta_x_distance ** 2 + delta_y_distance ** 2)
                
                if distance < 0.2:
                    self.get_logger().info(f'Hit!')
                    self.collision_flag = True
                
                # Check for collision risk based on distance
                if (prev_position_dynamic.x >= position.x or prev_position_dynamic.y <= position.y) and distance < 5.0:
                    swerve_data = Float32MultiArray()
                    swerve_x = 0.8 * (1 if prev_position_dynamic.x < position.x else -1)
                    swerve_y = -1*((distance / 5.0) * 0.8)
                    swerve_data.data = [position.x, position.y, swerve_x, swerve_y] 
                    
                    self.get_logger().info(f'Collision risk detected! Distance: {distance:.2f}')
                    self.collision_publisher.publish(swerve_data)
                else:
                    self.get_logger().info(f'No collision risk detected.')
                    self.no_collision_publisher.publish(position)
            
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

