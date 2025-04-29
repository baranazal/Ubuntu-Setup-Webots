#!/usr/bin/env python3

"""
REST API for AMR status and control
This module provides a Flask API to monitor and control AMR robots
"""

import threading
import time
import json
import math
import rclpy
from rclpy.node import Node
from flask import Flask, jsonify, request
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Global state dictionaries with thread locks
robot_status = {}
status_lock = threading.Lock()

nav_status = {}
nav_lock = threading.Lock()

# Navigation tolerance parameters
position_tolerance = 0.01
collision_distance = 0

class AMRNavigator(Node):
    """ROS2 node that handles AMR navigation commands"""
    
    def __init__(self):
        super().__init__('amr_navigator')
        
        self.cmd_vel_publishers = {}
        
        robot_names = ["AMR", "AMR2", "AMR3", "AMR4", "AMR5"]
        
        for robot_name in robot_names:
            qos = QoSProfile(depth=10)
            qos.reliability = ReliabilityPolicy.RELIABLE
            
            self.cmd_vel_publishers[robot_name] = self.create_publisher(
                Twist,
                f'/{robot_name}/cmd_vel',
                qos
            )
            
            with nav_lock:
                nav_status[robot_name] = {
                    "active": False,
                    "target_coordinates": None,
                    "status": "idle",
                    "last_updated": time.time()
                }
            
            self.get_logger().info(f"Navigation control ready for {robot_name}")
        
        self.nav_timer = self.create_timer(0.1, self.navigation_loop)
    
    def send_velocity_command(self, robot_name, linear_x, linear_y, angular_z):
        """Send a velocity command to a specific robot"""
        if robot_name not in self.cmd_vel_publishers:
            self.get_logger().error(f"Cannot send command - robot {robot_name} not found")
            return False
        
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_z
        
        self.cmd_vel_publishers[robot_name].publish(twist)
        return True
    
    def stop_robot(self, robot_name):
        """Send a stop command to a specific robot"""
        return self.send_velocity_command(robot_name, 0.0, 0.0, 0.0)
    
    def start_navigation_to_coordinates(self, robot_name, x, y, z):
        """Start navigation for a robot to specific coordinates"""
        with nav_lock:
            if robot_name not in nav_status:
                self.get_logger().error(f"Robot {robot_name} not found for navigation")
                return False
            
            nav_status[robot_name] = {
                "active": True,
                "target_coordinates": {"x": x, "y": y, "z": z},
                "status": "starting",
                "last_updated": time.time(),
                "error": None
            }
            
            self.get_logger().info(f"Starting navigation for {robot_name} to coordinates ({x}, {y}, {z})")
            return True
    
    def is_path_clear(self, robot_name, target_x, target_y):
        """Check if there are any robots in the path to the target coordinates"""
        with status_lock:
            if robot_name not in robot_status:
                return False
            
            # Get robot and target positions
            robot_x = robot_status[robot_name]["position"]["x"]
            robot_y = robot_status[robot_name]["position"]["y"]
            
            # Calculate direction vector
            dx = target_x - robot_x
            dy = target_y - robot_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Normalize direction vector
            if distance > 0:
                dx = dx / distance
                dy = dy / distance
            
            # Check for potential collisions with other robots
            for other_robot, status in robot_status.items():
                if other_robot == robot_name:
                    continue
                
                other_x = status["position"]["x"]
                other_y = status["position"]["y"]
                
                # Calculate vector from robot to other robot
                rdx = other_x - robot_x
                rdy = other_y - robot_y
                r_distance = math.sqrt(rdx*rdx + rdy*rdy)
                
                if r_distance < collision_distance:
                    # Too close to another robot
                    return False
                
                # Project the other robot's position onto the path
                if r_distance > 0:
                    # Calculate dot product
                    dot = rdx * dx + rdy * dy
                    
                    # Only consider robots ahead in the path
                    if 0 < dot < distance:
                        # Calculate closest point on path to other robot
                        closest_x = robot_x + dx * dot
                        closest_y = robot_y + dy * dot
                        
                        # Calculate distance from other robot to path
                        path_distance = math.sqrt((other_x - closest_x)**2 + (other_y - closest_y)**2)
                        
                        if path_distance < collision_distance:
                            return False
            
            return True
    
    def navigation_loop(self):
        """Main navigation control loop"""
        with nav_lock:
            for robot_name, status in nav_status.items():
                if not status["active"]:
                    continue
                
                # Get target position from coordinates
                if "target_coordinates" not in status or not status["target_coordinates"]:
                    self.get_logger().error(f"No valid target for {robot_name}")
                    status["active"] = False
                    status["status"] = "error"
                    status["error"] = "No valid target"
                    self.stop_robot(robot_name)
                    continue
                
                target_x = status["target_coordinates"]["x"]
                target_y = status["target_coordinates"]["y"]
                target_z = status["target_coordinates"].get("z", 0.0)
                
                # Get robot position
                with status_lock:
                    if robot_name not in robot_status:
                        self.get_logger().error(f"Robot {robot_name} position unknown")
                        status["active"] = False
                        status["status"] = "error"
                        status["error"] = "Robot position unknown"
                        continue
                        
                    robot_pos = robot_status[robot_name]["position"]
                    robot_x = robot_pos["x"]
                    robot_y = robot_pos["y"]
                    robot_theta = robot_status[robot_name]["orientation"]
                    robot_moving = robot_status[robot_name]["moving"]
                
                # Calculate distance to target
                dx = target_x - robot_x
                dy = target_y - robot_y
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Calculate angle to target
                target_angle = math.atan2(dy, dx)
                angle_diff = target_angle - robot_theta
        
                # Normalize to [-pi, pi]
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
        
                # Update status with navigation progress
                target_desc = f"coordinates ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})"
                status["status"] = f"navigating to {target_desc} - distance: {distance:.2f}m, angle error: {angle_diff:.2f}rad"
                status["last_updated"] = time.time()
                
                # Check if target reached
                if distance < position_tolerance:
                    self.get_logger().info(f"{robot_name} reached target {target_desc}")
                    status["active"] = False
                    status["status"] = "reached"
                    self.stop_robot(robot_name)
                    continue
                
                # Check if path is clear
                if not self.is_path_clear(robot_name, target_x, target_y):
                    self.get_logger().warn(f"Path not clear for {robot_name}")
                    status["status"] = "waiting - path blocked"
                    self.stop_robot(robot_name)
                    continue
                
                # Navigation constants
                linear_speed = 0.2
                angular_speed = 0.5
                angle_tolerance = 0.1
                fine_positioning_distance = 0.5
                
                # Navigate using two-phase approach (regular + fine positioning)
                if distance < fine_positioning_distance and distance > position_tolerance:
                    # Fine positioning mode
                    base_speed = 0.05
                    precision_speed = 0.1
                    fine_speed = base_speed + precision_speed * (distance / fine_positioning_distance)
                    
                    if abs(angle_diff) > angle_tolerance / 2:
                        angular_vel = (angular_speed * 0.5) * angle_diff / math.pi
                        self.send_velocity_command(robot_name, 0.0, 0.0, angular_vel)
                        status["status"] = f"fine positioning - aligning precisely: {angle_diff:.3f}rad"
                    else:
                        small_correction = angle_diff * 0.3
                        self.send_velocity_command(robot_name, fine_speed, 0.0, small_correction)
                        status["status"] = f"fine positioning - moving precisely: {distance:.3f}m"
                        self.get_logger().debug(f"{robot_name} fine positioning: current=({robot_x:.3f}, {robot_y:.3f}), target=({target_x:.3f}, {target_y:.3f})")
                
                elif abs(angle_diff) > angle_tolerance:
                    # Regular turning phase
                    angular_vel = angular_speed * (angle_diff / abs(angle_diff))
                    self.send_velocity_command(robot_name, 0.0, 0.0, angular_vel)
                    status["status"] = f"turning - angle error: {angle_diff:.2f}rad"
                else:
                    # Regular movement phase
                    min_regular_speed = 0.1
                    regular_adjusted_speed = max(min_regular_speed, min(linear_speed, linear_speed * distance))
                    angular_correction = angle_diff * 0.5
                    self.send_velocity_command(robot_name, regular_adjusted_speed, 0.0, angular_correction)
                    status["status"] = f"moving - distance: {distance:.2f}m"

class AMRStatusMonitor(Node):
    """ROS2 node that subscribes to AMR odometry topics and stores their status"""
    
    def __init__(self):
        super().__init__('amr_status_monitor')
        
        self.odom_subscribers = {}
        robot_names = ["AMR", "AMR2", "AMR3", "AMR4", "AMR5"]
        
        for robot_name in robot_names:
            with status_lock:
                robot_status[robot_name] = {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": 0.0,
                    "moving": False,
                    "last_updated": time.time()
                }
            
            self.odom_subscribers[robot_name] = self.create_subscription(
                Odometry,
                f'/{robot_name}/odom',
                lambda msg, name=robot_name: self.odom_callback(msg, name),
                10
            )
            
            self.get_logger().info(f"Monitoring status for {robot_name}")
    
    def odom_callback(self, msg, robot_name):
        """Process odometry data for a specific robot"""
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        pos_z = msg.pose.pose.position.z
        
        orientation = 2.0 * math.acos(msg.pose.pose.orientation.w)
        if msg.pose.pose.orientation.z < 0:
            orientation = -orientation
            
        linear_speed = math.sqrt(
            msg.twist.twist.linear.x**2 + 
            msg.twist.twist.linear.y**2
        )
        angular_speed = abs(msg.twist.twist.angular.z)
        is_moving = linear_speed > 0.01 or angular_speed > 0.01
        
        with status_lock:
            robot_status[robot_name] = {
                "position": {"x": pos_x, "y": pos_y, "z": pos_z},
                "orientation": orientation,
                "moving": is_moving,
                "last_updated": time.time(),
                "velocities": {
                    "linear": {
                        "x": msg.twist.twist.linear.x,
                        "y": msg.twist.twist.linear.y
                    },
                    "angular": {
                        "z": msg.twist.twist.angular.z
                    }
                }
            }

# Create Flask app
app = Flask(__name__)

@app.route('/api/status/<robot_id>', methods=['GET'])
def get_status(robot_id):
    """API endpoint to get the status of a specific AMR robot"""
    with status_lock:
        if robot_id in robot_status:
            status = robot_status[robot_id]
            
            nav_info = {}
            with nav_lock:
                if robot_id in nav_status:
                    robot_nav = nav_status[robot_id]
                    nav_info = {
                        "active": robot_nav["active"],
                        "status": robot_nav["status"]
                    }
                    
                    # Include coordinates if using direct coordinate navigation
                    if "target_coordinates" in robot_nav and robot_nav["target_coordinates"]:
                        nav_info["target_coordinates"] = robot_nav["target_coordinates"]
            
            return jsonify({
                "robot_id": robot_id,
                "position": status["position"],
                "orientation": status["orientation"],
                "moving": status["moving"],
                "last_updated": status["last_updated"],
                "navigation": nav_info
            })
        else:
            return jsonify({
                "error": f"Robot {robot_id} not found",
                "available_robots": list(robot_status.keys())
            }), 404

@app.route('/api/redirect', methods=['POST'])
def redirect_robot():
    """API endpoint to redirect a robot to specific coordinates"""
    global navigator_node

    data = request.json
    
    if not data:
        return jsonify({"error": "Missing request body"}), 400
    
    # Check for required field robot_id and coordinates
    required_fields = ['robot_id']
    for field in required_fields:
        if field not in data:
            return jsonify({"error": f"Missing required field: {field}"}), 400
    
    # Check that at least x and y coordinates are provided
    if 'x' not in data or 'y' not in data:
        return jsonify({"error": "Missing coordinate fields: x and y are required"}), 400
    
    robot_id = data['robot_id']
    target_x = float(data['x'])
    target_y = float(data['y'])
    
    # z coordinate (not used in 2D navigation but included for future compatibility)
    if 'z' in data:
        target_z = float(data['z'])
    else:
        target_z = 0.0
    
    with status_lock:
        if robot_id not in robot_status:
            return jsonify({
                "error": f"Robot {robot_id} not found",
                "available_robots": list(robot_status.keys())
            }), 404
    
    # Start navigation to the specified coordinates
    if navigator_node and navigator_node.start_navigation_to_coordinates(robot_id, target_x, target_y, target_z):
        return jsonify({
            "status": "success",
            "message": f"Robot {robot_id} is being redirected to coordinates ({target_x}, {target_y}, {target_z})",
            "robot_id": robot_id,
            "target_position": {"x": target_x, "y": target_y, "z": target_z}
        })
    else:
        return jsonify({
            "error": "Failed to start navigation",
            "status": "failed"
        }), 500

def run_flask_server():
    """Run the Flask server"""
    app.run(host='0.0.0.0', port=5000, debug=False)

def main():
    """Main function to run the AMR API server"""
    rclpy.init()
    
    monitor_node = AMRStatusMonitor()
    global navigator_node
    navigator_node = AMRNavigator()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(monitor_node)
    executor.add_node(navigator_node)
    
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    flask_thread = threading.Thread(target=run_flask_server, daemon=True)
    flask_thread.start()
    
    try:
        while ros_thread.is_alive() and flask_thread.is_alive():
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        executor.shutdown()
        monitor_node.destroy_node()
        navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 