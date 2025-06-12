#!/usr/bin/env python3

"""
AMR Waypoint Navigator

This script provides a simple waypoint navigation system for AMR robots in the Webots simulation.
It allows sending a robot to a sequence of waypoints, with basic motion control.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import time


class AMRWaypointNavigator(Node):
    """A simple waypoint navigation system for AMR robots."""
    
    def __init__(self, robot_name='AMR'):
        """Initialize the navigator with a robot name"""
        # Use robot name as the node name
        super().__init__(f'{robot_name}_navigator')
        
        # Robot identification
        self.robot_name = robot_name
        self.get_logger().info(f'Navigator initialized for robot: {self.robot_name}')
        
        # Navigation parameters
        self.position_tolerance = 0.1  # How close to get to waypoint (meters)
        self.angle_tolerance = 0.1  # How precise to get angle (radians)
        self.linear_speed = 0.2  # Default linear speed (m/s)
        self.angular_speed = 0.5  # Default angular speed (rad/s)
        self.rate = 10  # Control loop rate (Hz)
        
        # Waypoint management
        self.waypoints = []
        self.current_waypoint_index = 0
        self.is_navigating = False
        
        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.position_initialized = False
        
        # Create publisher for velocity commands
        self.vel_pub = self.create_publisher(
            Twist,
            f'/{self.robot_name}/cmd_vel',
            10
        )
        
        # Create subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            10
        )
        
        # Timer for navigation control loop
        self.timer = self.create_timer(1.0/self.rate, self.navigation_loop)
        
        self.get_logger().info('Waiting for odometry data...')
    
    def odom_callback(self, msg):
        """Callback for receiving odometry data"""
        # Extract position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract orientation (yaw/theta)
        # Convert quaternion to Euler angles
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Compute yaw (theta) from quaternion
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)
        
        if not self.position_initialized:
            self.position_initialized = True
            self.get_logger().info(f'Initial position: ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_theta:.2f})')
    
    def set_waypoints(self, waypoints):
        """Set a list of waypoints to navigate to
        
        Args:
            waypoints: List of (x, y) tuples representing waypoints
        """
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.is_navigating = True
        self.get_logger().info(f'Set {len(waypoints)} waypoints for navigation')
        for i, (x, y) in enumerate(waypoints):
            self.get_logger().info(f'Waypoint {i}: ({x:.2f}, {y:.2f})')
    
    def navigation_loop(self):
        """Main control loop for navigation"""
        if not self.is_navigating or not self.position_initialized:
            return
        
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('Reached final waypoint!')
            self.is_navigating = False
            self.stop_robot()
            return
        
        # Get current waypoint
        target_x, target_y = self.waypoints[self.current_waypoint_index]
        
        # Calculate distance to waypoint
        distance = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
        
        # If we've reached the waypoint, move to the next one
        if distance < self.position_tolerance:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}: ({target_x:.2f}, {target_y:.2f})')
            self.current_waypoint_index += 1
            self.stop_robot()
            return
        
        # Calculate angle to waypoint
        target_angle = math.atan2(target_y - self.current_y, target_x - self.current_x)
        
        # Calculate the shortest angle difference
        angle_diff = target_angle - self.current_theta
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Create velocity command
        cmd_vel = Twist()
        
        # If not facing the right direction, rotate first
        if abs(angle_diff) > self.angle_tolerance:
            # Rotate toward target
            cmd_vel.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            self.vel_pub.publish(cmd_vel)
            return
        
        # Now we're facing the right direction, move forward
        cmd_vel.linear.x = self.linear_speed
        self.vel_pub.publish(cmd_vel)
    
    def stop_robot(self):
        """Stop the robot by sending zero velocity"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.angular.z = 0.0
        self.vel_pub.publish(cmd_vel)
        self.get_logger().debug('Robot stopped')
    
    def get_status(self):
        """Get the current navigation status
        
        Returns:
            dict: Status information with current position, target, etc.
        """
        status = {
            'robot_name': self.robot_name,
            'current_position': (self.current_x, self.current_y, self.current_theta),
            'is_navigating': self.is_navigating,
            'waypoints_remaining': len(self.waypoints) - self.current_waypoint_index if self.is_navigating else 0
        }
        
        if self.is_navigating and self.current_waypoint_index < len(self.waypoints):
            status['current_target'] = self.waypoints[self.current_waypoint_index]
            
        return status


def main(args=None):
    """Main entry point for the waypoint navigator"""
    rclpy.init(args=args)
    
    # Default robot name
    robot_name = 'AMR'
    
    # Create navigator
    navigator = AMRWaypointNavigator(robot_name)
    
    # Example usage - set some waypoints for testing
    waypoints = [
        (1.0, 1.0),    # Move to x=1, y=1
        (2.0, 0.0),    # Move to x=2, y=0
        (0.0, 0.0)     # Return to origin
    ]
    
    # Let the robot reach each waypoint
    navigator.set_waypoints(waypoints)
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Navigation interrupted')
    finally:
        # Stop the robot before shutting down
        navigator.stop_robot()
        navigator.destroy_node()
        rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    main() 