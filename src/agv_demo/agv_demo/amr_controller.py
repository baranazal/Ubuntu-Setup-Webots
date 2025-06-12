#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from controller import Robot
import sys

class AMRController(Node):
    """
    Controller for an AMR (Autonomous Mobile Robot) with mecanum wheels.
    Handles the interface between ROS2 and Webots simulation.
    """
    def __init__(self):
        # Initialize the robot
        try:
            self.robot = Robot()
            self.robot_name = self.robot.getName()
            
            # Initialize as a ROS2 node with unique name
            super().__init__(f'amr_controller_{self.robot_name}')
            
            # Get the timestep
            self.timestep = int(self.robot.getBasicTimeStep())
            self.get_logger().info(f'Controlling robot: {self.robot_name} with timestep {self.timestep}ms')
            
            # Initialize mecanum wheel motors and catch any initialization errors
            self._init_motors()
            
            # ROS2 subscriber for velocity commands
            self.cmd_vel_subscriber = self.create_subscription(
                Twist, f'/{self.robot_name}/cmd_vel', self.cmd_vel_callback, 10)
            
            # Log the topic we're subscribed to
            self.get_logger().info(f'Subscribed to velocity commands on topic: /{self.robot_name}/cmd_vel')
            
            # Timer for main control loop
            self.control_loop_timer = self.create_timer(self.timestep / 1000.0, self.control_loop)
            
            # Robot parameters
            self.wheel_radius = 0.055
            self.wheel_distance_x = 0.34  # meters (distance between left and right wheels)
            self.wheel_distance_y = 0.30  # meters (distance between front and rear wheels)
            self.max_wheel_speed = 15.0  # rad/s
            
            # State tracking
            self.last_cmd_time = self.get_clock().now()
            self.cmd_timeout = 0.5  # seconds
            
            self.get_logger().info(f'AMR Controller for {self.robot_name} initialized successfully')
        except Exception as e:
            if hasattr(self, 'get_logger'):
                self.get_logger().error(f'Error initializing AMR controller: {str(e)}')
            else:
                print(f'Error initializing AMR controller: {str(e)}')
            sys.exit(1)
    
    def _init_motors(self):
        """Initialize wheel motors and set up velocity control mode"""
        try:
            # Initialize wheel motors
            self.front_left_motor = self.robot.getDevice('front_left_motor')
            self.front_right_motor = self.robot.getDevice('front_right_motor')
            self.rear_left_motor = self.robot.getDevice('rear_left_motor')
            self.rear_right_motor = self.robot.getDevice('rear_right_motor')
            
            # Ensure all motors are found
            if not all([self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor]):
                missing = []
                if not self.front_left_motor: missing.append('front_left_motor')
                if not self.front_right_motor: missing.append('front_right_motor')
                if not self.rear_left_motor: missing.append('rear_left_motor')
                if not self.rear_right_motor: missing.append('rear_right_motor')
                
                error_msg = f"Missing motors: {', '.join(missing)}"
                self.get_logger().error(error_msg)
                raise RuntimeError(error_msg)
            
            # Set motors to velocity control mode
            for motor in [self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor]:
                motor.setPosition(float('inf'))
                motor.setVelocity(0.0)
                
            return True
        except Exception as e:
            self.get_logger().error(f'Error initializing motors: {str(e)}')
            raise
    
    def cmd_vel_callback(self, msg):
        """
        Process velocity commands received through ROS2
        Args:
            msg: Twist message containing linear and angular velocity commands
        """
        try:
            vx = msg.linear.x   # Forward velocity
            vy = msg.linear.y   # Lateral velocity (for mecanum wheels)
            omega = msg.angular.z  # Angular velocity
            
            # Update the timestamp of the last received command
            self.last_cmd_time = self.get_clock().now()
            
            # Set wheel speeds based on desired motion
            self.set_mecanum_wheel_speeds(vx, vy, omega)
        except Exception as e:
            self.get_logger().error(f'Error processing velocity command: {str(e)}')
    
    def set_mecanum_wheel_speeds(self, vx, vy, omega):
        """
        Set wheel speeds for mecanum drive motion
        Args:
            vx: forward velocity (m/s)
            vy: lateral velocity (m/s)
            omega: angular velocity (rad/s)
        """
        try:
            # Calculate wheel speeds based on mecanum kinematics
            front_left = (vx - vy - omega * (self.wheel_distance_x + self.wheel_distance_y) / 2.0) / self.wheel_radius
            front_right = (vx + vy + omega * (self.wheel_distance_x + self.wheel_distance_y) / 2.0) / self.wheel_radius
            rear_left = (vx + vy - omega * (self.wheel_distance_x + self.wheel_distance_y) / 2.0) / self.wheel_radius
            rear_right = (vx - vy + omega * (self.wheel_distance_x + self.wheel_distance_y) / 2.0) / self.wheel_radius
            
            # Apply velocity limits
            front_left = max(min(front_left, self.max_wheel_speed), -self.max_wheel_speed)
            front_right = max(min(front_right, self.max_wheel_speed), -self.max_wheel_speed)
            rear_left = max(min(rear_left, self.max_wheel_speed), -self.max_wheel_speed)
            rear_right = max(min(rear_right, self.max_wheel_speed), -self.max_wheel_speed)
            
            # Set motor velocities
            self.front_left_motor.setVelocity(front_left)
            self.front_right_motor.setVelocity(front_right)
            self.rear_left_motor.setVelocity(rear_left)
            self.rear_right_motor.setVelocity(rear_right)
            
        except Exception as e:
            self.get_logger().error(f'Error setting wheel speeds: {str(e)}')
            self.stop_robot()
    
    def stop_robot(self):
        """Emergency stop function to halt all motors"""
        try:
            for motor in [self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor]:
                motor.setVelocity(0.0)
            self.get_logger().warn(f'Emergency stop triggered for {self.robot_name}')
        except Exception as e:
            self.get_logger().error(f'Error stopping robot: {str(e)}')
    
    def check_command_timeout(self):
        """Check if we've received a command recently, stop robot if command timeout"""
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_cmd_time).nanoseconds / 1e9
        
        if time_diff > self.cmd_timeout:
            # No recent commands, stop the robot for safety
            self.stop_robot()
            return True
        return False
    
    def control_loop(self):
        """Main control loop - step the simulation and apply safety checks"""
        try:
            # Step the simulation
            if self.robot.step(self.timestep) == -1:
                # Simulation ended
                self.get_logger().info('Simulation ended, shutting down')
                return False
            
            # Check for command timeout
            self.check_command_timeout()
            
            return True
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {str(e)}')
            self.stop_robot()
            return False

def main(args=None):
    """Main function to start the AMR controller"""
    try:
        rclpy.init(args=args)
        controller = AMRController()
        
        try:
            rclpy.spin(controller)
        except KeyboardInterrupt:
            controller.get_logger().info('Keyboard interrupt received, shutting down')
        except Exception as e:
            controller.get_logger().error(f'Error during execution: {str(e)}')
        finally:
            # Clean up
            controller.stop_robot()
            controller.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        print(f'Fatal error: {str(e)}')
        sys.exit(1)

if __name__ == '__main__':
    main()
