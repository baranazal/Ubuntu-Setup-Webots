#!/usr/bin/env python3

"""
Conveyor Belt Controller for Webots
"""

import sys
import time
import os
from controller import Robot
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, String
import json

class ConveyorController:
    def __init__(self, robot):
        self.robot = robot
        self.name = robot.getName()
        self.timestep = int(robot.getBasicTimeStep())
        
        # Belt parameters
        self.belt_speed = 0.5
        self.belt_active = True
        self.belt_direction = 1
        
        # Init ROS node
        rclpy.init(args=None)
        self.node = rclpy.create_node(f'{self.name}_controller')
        
        # Create subscribers
        self.active_sub = self.node.create_subscription(
            Bool, f'/{self.name}/set_active', self.set_active_callback, 10)
        self.speed_sub = self.node.create_subscription(
            Float64, f'/{self.name}/set_speed', self.set_speed_callback, 10)
        self.direction_sub = self.node.create_subscription(
            Float64, f'/{self.name}/set_direction', self.set_direction_callback, 10)
        
        # Create publisher
        self.status_pub = self.node.create_publisher(
            String, f'/{self.name}/status', 10)
        
        # Find motors
        self.motors = []
        for i in range(robot.getNumberOfDevices()):
            device = robot.getDeviceByIndex(i)
            if device and device.getType() == Robot.MOTOR:
                self.motors.append(device)
                device.setPosition(float('inf'))
                device.setVelocity(0.0)
        
        self.last_status_time = time.time()
    
    def set_active_callback(self, msg):
        self.belt_active = msg.data
        self.update_motors()
        self.publish_status()
    
    def set_speed_callback(self, msg):
        self.belt_speed = abs(msg.data)
        self.update_motors()
        self.publish_status()
    
    def set_direction_callback(self, msg):
        self.belt_direction = 1 if msg.data > 0 else -1
        self.update_motors()
        self.publish_status()
    
    def update_motors(self):
        if self.motors:
            for motor in self.motors:
                if self.belt_active:
                    velocity = self.belt_speed * self.belt_direction
                    motor.setVelocity(velocity)
                else:
                    motor.setVelocity(0.0)
    
    def publish_status(self):
        try:
            status = {
                "name": self.name,
                "active": self.belt_active,
                "speed": self.belt_speed,
                "direction": self.belt_direction,
                "timestamp": time.time()
            }
            
            msg = String()
            msg.data = json.dumps(status)
            self.status_pub.publish(msg)
        except Exception as e:
            print(f"Error publishing status: {str(e)}")
    
    def run(self):
        while self.robot.step(self.timestep) != -1:
            rclpy.spin_once(self.node, timeout_sec=0)
            
            if time.time() - self.last_status_time >= 5.0:
                self.publish_status()
                self.last_status_time = time.time()

def main():
    robot = Robot()
    controller = ConveyorController(robot)
    
    try:
        controller.run()
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        if controller.node:
            controller.node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()