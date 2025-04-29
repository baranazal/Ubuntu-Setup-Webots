#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, String
import json

class ConveyorInterface(Node):
    def __init__(self):
        super().__init__('conveyor_interface')
        
        # Parameters
        self.declare_parameter('conveyor_id', 'MainConveyor')
        self.conveyor_id = self.get_parameter('conveyor_id').get_parameter_value().string_value
        
        # Publishers
        self.active_pub = self.create_publisher(
            Bool, f'/{self.conveyor_id}/set_active', 10)
        self.speed_pub = self.create_publisher(
            Float64, f'/{self.conveyor_id}/set_speed', 10)
        self.direction_pub = self.create_publisher(
            Float64, f'/{self.conveyor_id}/set_direction', 10)
        
        # Subscriber
        self.status_sub = self.create_subscription(
            String, f'/{self.conveyor_id}/status', self.status_callback, 10)
        
        self.get_logger().info(f'Conveyor interface initialized for {self.conveyor_id}')
    
    def status_callback(self, msg):
        try:
            status = json.loads(msg.data)
            self.get_logger().debug(f"Received status: {status}")
        except Exception as e:
            self.get_logger().error(f"Error parsing status: {str(e)}")
    
    def set_active(self, active):
        msg = Bool()
        msg.data = active
        self.active_pub.publish(msg)
        self.get_logger().info(f"Set conveyor active: {active}")
    
    def set_speed(self, speed):
        msg = Float64()
        msg.data = speed
        self.speed_pub.publish(msg)
        self.get_logger().info(f"Set conveyor speed: {speed}")
    
    def set_direction(self, direction):
        msg = Float64()
        msg.data = direction
        self.direction_pub.publish(msg)
        self.get_logger().info(f"Set conveyor direction: {direction}")

def main(args=None):
    rclpy.init(args=args)
    interface = ConveyorInterface()
    
    try:
        rclpy.spin(interface)
    except KeyboardInterrupt:
        pass
    finally:
        interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()