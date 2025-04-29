#!/usr/bin/env python3

"""
Simple AMR Navigator

A standalone script for navigating AMR robots in the Webots simulation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import sys
import argparse

class SimpleNavigator(Node):
    """A simple navigator for AMR robots."""
    
    def __init__(self, robot_name='AMR'):
        super().__init__(f'{robot_name}_navigator')
        
        # Robot identification
        self.robot_name = robot_name
        self.get_logger().info(f'Navigator initialized for robot: {self.robot_name}')
        
        # Navigation parameters
        self.position_tolerance = 0.1  # meters
        self.angle_tolerance = 0.1  # radians
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        
        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.position_initialized = False
        
        # Waypoint management
        self.waypoints = []
        self.current_waypoint_index = 0
        self.is_navigating = False
        
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
        
        # Create timer for navigation control
        self.timer = self.create_timer(0.1, self.navigation_loop)
        
        self.get_logger().info('Waiting for odometry data...')
    
    def odom_callback(self, msg):
        """Callback for receiving odometry data"""
        # Extract position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract orientation (yaw/theta)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Convert quaternion to Euler angle (yaw)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)
        
        if not self.position_initialized:
            self.position_initialized = True
            self.get_logger().info(f'Initial position: ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_theta:.2f})')
    
    def set_waypoints(self, waypoints):
        """Set a list of waypoints to navigate to"""
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.is_navigating = True
        self.get_logger().info(f'Set {len(waypoints)} waypoints for navigation')
        for i, (x, y) in enumerate(waypoints):
            self.get_logger().info(f'Waypoint {i+1}: ({x:.2f}, {y:.2f})')
    
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
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index+1}: ({target_x:.2f}, {target_y:.2f})')
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

def parse_waypoints(waypoints_str):
    """Parse waypoints from a string with format 'x1,y1;x2,y2;...'"""
    waypoints = []
    
    if not waypoints_str:
        return waypoints
    
    # Split by semicolon to get individual waypoints
    waypoint_strs = waypoints_str.split(';')
    
    for wp_str in waypoint_strs:
        # Split by comma to get x and y
        try:
            x, y = map(float, wp_str.split(','))
            waypoints.append((x, y))
        except ValueError:
            print(f"Error: Invalid waypoint format: {wp_str}")
            continue
    
    return waypoints

def main(args=None):
    parser = argparse.ArgumentParser(description='Simple navigator for AMR robots')
    parser.add_argument('--robot', '-r', type=str, default='AMR', 
                      help='Robot name (default: AMR)')
    parser.add_argument('--waypoints', '-w', type=str, required=True,
                      help='Waypoints in format "x1,y1;x2,y2;..." (example: "1.0,1.0;2.0,0.0;0.0,0.0")')
    parser.add_argument('--speed', '-s', type=float, default=0.2,
                      help='Linear movement speed in m/s (default: 0.2)')
    parser.add_argument('--angular-speed', '-a', type=float, default=0.5,
                      help='Angular rotation speed in rad/s (default: 0.5)')
    parser.add_argument('--tolerance', '-t', type=float, default=0.1,
                      help='Position tolerance in meters (default: 0.1)')
    
    parsed_args = parser.parse_args(args)
    
    # Parse waypoints
    waypoints = parse_waypoints(parsed_args.waypoints)
    
    if not waypoints:
        print("Error: No valid waypoints provided")
        parser.print_help()
        return 1
    
    print(f"Initializing navigation for robot '{parsed_args.robot}' with {len(waypoints)} waypoints:")
    for i, (x, y) in enumerate(waypoints):
        print(f"  Waypoint {i+1}: ({x:.2f}, {y:.2f})")
    
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create navigator
    navigator = SimpleNavigator(parsed_args.robot)
    
    # Configure navigator
    navigator.linear_speed = parsed_args.speed
    navigator.angular_speed = parsed_args.angular_speed
    navigator.position_tolerance = parsed_args.tolerance
    
    # Set waypoints and start navigation
    navigator.set_waypoints(waypoints)
    
    print(f"Robot {parsed_args.robot} is now navigating to waypoints.")
    print("Press Ctrl+C to interrupt...")
    
    try:
        # Run the navigation
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        print("\nNavigation interrupted by user")
    finally:
        # Stop the robot
        navigator.stop_robot()
        navigator.destroy_node()
        rclpy.shutdown()
    
    return 0

if __name__ == '__main__':
    sys.exit(main()) 