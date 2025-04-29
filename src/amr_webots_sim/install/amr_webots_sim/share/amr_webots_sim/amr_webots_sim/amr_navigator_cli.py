#!/usr/bin/env python3

"""
AMR Navigator CLI

Command-line interface for sending AMR robots to waypoints
"""

import rclpy
import argparse
import sys
import time
from amr_webots_sim.amr_waypoint_navigator import AMRWaypointNavigator

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
    """Main entry point for the navigator CLI"""
    parser = argparse.ArgumentParser(description='Command-line interface for AMR waypoint navigation')
    parser.add_argument('--robot', '-r', type=str, default='AMR', 
                        help='Robot name (default: AMR)')
    parser.add_argument('--waypoints', '-w', type=str, 
                        help='Waypoints in format "x1,y1;x2,y2;..." (example: "1.0,1.0;2.0,0.0;0.0,0.0")')
    parser.add_argument('--speed', '-s', type=float, default=0.2,
                        help='Linear movement speed in m/s (default: 0.2)')
    parser.add_argument('--angular-speed', '-a', type=float, default=0.5,
                        help='Angular rotation speed in rad/s (default: 0.5)')
    parser.add_argument('--tolerance', '-t', type=float, default=0.1,
                        help='Position tolerance in meters (default: 0.1)')
    
    parsed_args = parser.parse_args()
    
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
    navigator = AMRWaypointNavigator(parsed_args.robot)
    
    # Configure navigator
    navigator.linear_speed = parsed_args.speed
    navigator.angular_speed = parsed_args.angular_speed
    navigator.position_tolerance = parsed_args.tolerance
    
    # Set waypoints and start navigation
    navigator.set_waypoints(waypoints)
    
    print(f"Robot {parsed_args.robot} is now navigating to waypoints.")
    print("Press Ctrl+C to interrupt...")
    
    try:
        # Spin in the background
        while navigator.is_navigating:
            rclpy.spin_once(navigator, timeout_sec=0.1)
            time.sleep(0.1)  # Small sleep to prevent CPU hogging
            
        print(f"Robot {parsed_args.robot} has completed all waypoints!")
        
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