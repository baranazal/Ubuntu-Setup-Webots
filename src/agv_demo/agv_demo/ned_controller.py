#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math
import numpy as np

class NedROS2Controller(Node):
    """
    ROS2 Node for controlling the Ned robot arm in Webots.
    Acts as a bridge between ROS2 topics and the Webots controller.
    """
    def __init__(self):
        # Initialize the node
        super().__init__('ned_ros2_controller')
        
        # Default joint positions (home position in radians)
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Current joint positions
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gripper_position = 0.0
        
        # Create subscribers for user commands
        self.joint_commands_sub = self.create_subscription(
            Float64MultiArray, '/ned/joint_commands', self.joint_commands_callback, 10)
            
        self.gripper_command_sub = self.create_subscription(
            Float64MultiArray, '/ned/gripper_command', self.gripper_command_callback, 10)
        
        # Create subscribers for feedback from Webots controller
        self.joint_states_sub = self.create_subscription(
            JointState, '/ned/joint_states', self.joint_states_callback, 10)
        
        # Create publishers for sending commands (for the move_to_pose method)
        self.joint_commands_pub = self.create_publisher(
            Float64MultiArray, '/ned/joint_commands', 10)
        
        # Create periodic timer (can be used for monitoring)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info("Ned ROS2 Controller started. Waiting for commands...")
    
    def joint_states_callback(self, msg):
        """Process incoming joint states from the Webots controller"""
        try:
            # Update joint positions
            for i, name in enumerate(msg.name):
                if i < 6:  # Only the arm joints, not the gripper
                    self.current_joint_positions[i] = msg.position[i]
                elif 'gripper' in name:
                    # Gripper position
                    self.gripper_position = msg.position[i]
        except Exception as e:
            self.get_logger().error(f"Error processing joint states: {str(e)}")
    
    def joint_commands_callback(self, msg):
        """
        Process joint commands from users and forward to the Webots controller
        (Since our Webots controller directly subscribes to /ned/joint_commands,
        we just log the commands here for monitoring)
        """
        try:
            if len(msg.data) != 6:
                self.get_logger().warning(f"Expected 6 joint positions, got {len(msg.data)}")
                return
                
            # Log the command
            self.get_logger().info(f"Received joint command: {msg.data}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing joint command: {str(e)}")
    
    def gripper_command_callback(self, msg):
        """
        Process gripper commands from users and forward to the Webots controller
        (Since our Webots controller directly subscribes to /ned/gripper_command,
        we just log the commands here for monitoring)
        """
        try:
            if len(msg.data) != 1:
                self.get_logger().warning(f"Expected 1 gripper value, got {len(msg.data)}")
                return
                
            # Log the command
            self.get_logger().info(f"Received gripper command: {msg.data[0]}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing gripper command: {str(e)}")
    
    def timer_callback(self):
        """Periodic callback for monitoring or other tasks"""
        # We can add monitoring or other periodic tasks here if needed
        pass
    
    def move_to_pose(self, x, y, z, roll=0.0, pitch=math.pi/2, yaw=0.0):
        """
        Utility method to move the arm to a Cartesian pose
        This would need proper inverse kinematics calculations
        """
        self.get_logger().info(f"Moving to pose: x={x}, y={y}, z={z}")
        # For now, we just use a simple predefined position
        positions = [0.0, 0.2, -0.3, 0.0, 0.5, 0.0]
        
        # Create and publish message
        msg = Float64MultiArray()
        msg.data = positions
        self.joint_commands_pub.publish(msg)
        return True

def main(args=None):
    rclpy.init(args=args)
    ned_controller = NedROS2Controller()
    
    try:
        rclpy.spin(ned_controller)
    except KeyboardInterrupt:
        ned_controller.get_logger().info("Keyboard interrupt, shutting down")
    except Exception as e:
        ned_controller.get_logger().error(f"Error: {str(e)}")
    finally:
        ned_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 