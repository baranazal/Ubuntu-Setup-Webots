#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from controller import Robot
import sys
import math
import time
import os

class NedController(Node):
    """
    Controller for the Niryo Ned robotic arm.
    Handles the interface between ROS2 and Webots simulation.
    """
    def __init__(self):
        # Initialize the robot
        try:
            # Create robot instance with proper connection
            self.robot = Robot()
            self.robot_name = self.robot.getName()
            
            # Print connection information
            print(f"Successfully connected to robot '{self.robot_name}' in Webots")
            
            # Initialize as a ROS2 node with unique name
            super().__init__('ned_webots_controller')
            
            # Get the timestep
            self.timestep = int(self.robot.getBasicTimeStep())
            self.get_logger().info(f'Controlling robot: {self.robot_name} with timestep {self.timestep}ms')
            
            # Initialize joint motors
            self._init_motors()
            
            # Initialize gripper
            self._init_gripper()
            
            # ROS2 subscribers
            self.joint_cmd_subscriber = self.create_subscription(
                Float64MultiArray, '/ned/joint_commands', self.joint_cmd_callback, 10)
            
            self.gripper_cmd_subscriber = self.create_subscription(
                Float64MultiArray, '/ned/gripper_command', self.gripper_cmd_callback, 10)
            
            # ROS2 publishers
            self.joint_states_publisher = self.create_publisher(
                JointState, '/ned/joint_states', 10)
            
            # Timer for main control loop
            self.control_loop_timer = self.create_timer(self.timestep / 1000.0, self.control_loop)
            
            # Joint state message
            self.joint_state_msg = JointState()
            self.joint_state_msg.name = [
                'joint_1', 'joint_2', 'joint_3', 
                'joint_4', 'joint_5', 'joint_6',
                'gripper::left', 'gripper::right'
            ]
            
            self.get_logger().info(f'Ned Webots Controller initialized successfully.')
            self.get_logger().info(f'Subscribing to /ned/joint_commands and /ned/gripper_command')
            self.get_logger().info(f'Publishing to /ned/joint_states')
            
        except Exception as e:
            if hasattr(self, 'get_logger'):
                self.get_logger().error(f'Error initializing Ned controller: {str(e)}')
            else:
                print(f'Error initializing Ned controller: {str(e)}')
            sys.exit(1)
    
    def _init_motors(self):
        """Initialize joint motors and position sensors"""
        try:
            # Joint motors
            self.joint_motors = []
            self.joint_sensors = []
            
            # Get 6 arm joint motors and their sensors
            for i in range(1, 7):
                motor = self.robot.getDevice(f'joint_{i}')
                if motor is None:
                    raise RuntimeError(f"Could not find motor joint_{i}")
                motor.setPosition(0.0)  # Start in home position
                motor.setVelocity(1.0)  # Set initial velocity
                self.joint_motors.append(motor)
                
                sensor = self.robot.getDevice(f'joint_{i}_sensor')
                if sensor is None:
                    raise RuntimeError(f"Could not find position sensor joint_{i}_sensor")
                sensor.enable(self.timestep)
                self.joint_sensors.append(sensor)
                
            self.get_logger().info("Joint motors and sensors initialized")
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error initializing motors: {str(e)}')
            raise
    
    def _init_gripper(self):
        """Initialize gripper and its position sensors"""
        try:
            # Get gripper motor and sensor
            self.gripper_left = self.robot.getDevice("gripper::left")
            self.gripper_right = self.robot.getDevice("gripper::right")
            
            if self.gripper_left is None or self.gripper_right is None:
                raise RuntimeError("Could not find gripper motors")
                
            self.gripper_left_sensor = self.robot.getDevice("gripper::left_sensor")
            self.gripper_right_sensor = self.robot.getDevice("gripper::right_sensor")
            
            if self.gripper_left_sensor is None or self.gripper_right_sensor is None:
                raise RuntimeError("Could not find gripper sensors")
            
            # Enable sensors
            self.gripper_left_sensor.enable(self.timestep)
            self.gripper_right_sensor.enable(self.timestep)
            
            # Set initial position (closed gripper)
            self.gripper_left.setPosition(0.0)
            self.gripper_right.setPosition(0.0)
            
            self.get_logger().info("Gripper initialized")
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error initializing gripper: {str(e)}')
            raise
    
    def joint_cmd_callback(self, msg):
        """
        Process joint commands received through ROS2
        Args:
            msg: Float64MultiArray containing target joint positions
        """
        try:
            if len(msg.data) != 6:
                self.get_logger().warning(f"Expected 6 joint positions, got {len(msg.data)}")
                return
            
            self.get_logger().info(f"Received joint command: {msg.data}")
                
            for i, position in enumerate(msg.data):
                if i < len(self.joint_motors):
                    self.joint_motors[i].setPosition(position)
            
        except Exception as e:
            self.get_logger().error(f'Error processing joint command: {str(e)}')
    
    def gripper_cmd_callback(self, msg):
        """
        Process gripper commands received through ROS2
        Args:
            msg: Float64MultiArray with a single value 0-1 representing gripper position
                 0 = closed, 1 = fully open
        """
        try:
            if len(msg.data) != 1:
                self.get_logger().warning(f"Expected 1 gripper value, got {len(msg.data)}")
                return
                
            self.get_logger().info(f"Received gripper command: {msg.data[0]}")
                
            # Convert 0-1 range to actual position values
            # The gripper moves in opposite directions
            position = msg.data[0] * 0.01  # Scale value to match the gripper range (0 to 0.01)
            self.gripper_left.setPosition(position)
            self.gripper_right.setPosition(position)
            
        except Exception as e:
            self.get_logger().error(f'Error processing gripper command: {str(e)}')
    
    def publish_joint_states(self):
        """Publish current joint states to ROS2"""
        try:
            # Get current time for header
            now = self.get_clock().now().to_msg()
            
            # Update joint state message
            self.joint_state_msg.header.stamp = now
            
            # Get current positions
            positions = []
            velocities = []
            
            # Add arm joint values
            for sensor in self.joint_sensors:
                positions.append(sensor.getValue())
                velocities.append(0.0)  # We don't have velocity sensing in this simulation
            
            # Add gripper values
            positions.append(self.gripper_left_sensor.getValue())
            positions.append(self.gripper_right_sensor.getValue())
            velocities.append(0.0)
            velocities.append(0.0)
            
            # Update message
            self.joint_state_msg.position = positions
            self.joint_state_msg.velocity = velocities
            
            # Publish
            self.joint_states_publisher.publish(self.joint_state_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing joint states: {str(e)}')
    
    def control_loop(self):
        """Main control loop - step the simulation and publish sensor data"""
        try:
            # Step the simulation
            if self.robot.step(self.timestep) == -1:
                # Simulation ended
                self.get_logger().info('Simulation ended, shutting down')
                return False
            
            # Publish joint states
            self.publish_joint_states()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {str(e)}')
            return False

def main(args=None):
    """Main function to start the Ned controller"""
    try:
        # Print environment variables for debugging
        robot_name = os.environ.get('WEBOTS_ROBOT_NAME', 'unknown')
        controller_url = os.environ.get('WEBOTS_CONTROLLER_URL', 'unknown')
        print(f"Starting Ned controller for robot '{robot_name}' with URL '{controller_url}'")
        
        rclpy.init(args=args)
        controller = NedController()
        
        try:
            rclpy.spin(controller)
        except KeyboardInterrupt:
            controller.get_logger().info('Keyboard interrupt received, shutting down')
        except Exception as e:
            controller.get_logger().error(f'Error during execution: {str(e)}')
        finally:
            # Clean up
            controller.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        print(f'Fatal error: {str(e)}')
        sys.exit(1)

if __name__ == '__main__':
    main() 