#!/usr/bin/env python3

import rclpy
from std_msgs.msg import Float64, Bool, String
import json

class ConveyorDriver:
    def __init__(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__name = self.__robot.getName()
        
        # Get webots_node to enable clock
        self.__node = webots_node
        
        # Belt parameters
        self.belt_speed = 0.5
        self.belt_active = True
        self.belt_direction = 1
        
        # Create subscriptions
        self.__node.create_subscription(
            Bool, f'/{self.__name}/set_active', self.__set_active_callback, 10)
        self.__node.create_subscription(
            Float64, f'/{self.__name}/set_speed', self.__set_speed_callback, 10)
        self.__node.create_subscription(
            Float64, f'/{self.__name}/set_direction', self.__set_direction_callback, 10)
        
        # Create publisher
        self.__status_pub = self.__node.create_publisher(
            String, f'/{self.__name}/status', 10)
        
        # Find motors
        self.__motors = []
        for i in range(self.__robot.getNumberOfDevices()):
            device = self.__robot.getDeviceByIndex(i)
            if device and device.getType() == self.__robot.MOTOR:
                self.__motors.append(device)
                device.setPosition(float('inf'))
                device.setVelocity(0.0)
                
        self.__node.get_logger().info(f'Conveyor driver initialized for {self.__name}')
        
    def __set_active_callback(self, msg):
        self.belt_active = msg.data
        self.__update_motors()
        self.__publish_status()
        
    def __set_speed_callback(self, msg):
        self.belt_speed = abs(msg.data)
        self.__update_motors()
        self.__publish_status()
        
    def __set_direction_callback(self, msg):
        self.belt_direction = 1 if msg.data > 0 else -1
        self.__update_motors()
        self.__publish_status()
        
    def __update_motors(self):
        if self.__motors:
            for motor in self.__motors:
                if self.belt_active:
                    velocity = self.belt_speed * self.belt_direction
                    motor.setVelocity(velocity)
                else:
                    motor.setVelocity(0.0)
                    
    def __publish_status(self):
        try:
            status = {
                "name": self.__name,
                "active": self.belt_active,
                "speed": self.belt_speed,
                "direction": self.belt_direction,
                "timestamp": self.__node.get_clock().now().to_msg().sec
            }
            
            msg = String()
            msg.data = json.dumps(status)
            self.__status_pub.publish(msg)
        except Exception as e:
            self.__node.get_logger().error(f"Error publishing status: {str(e)}")
            
    def step(self):
        # This function is called at each timestep
        # We'll use it to publish status periodically
        pass