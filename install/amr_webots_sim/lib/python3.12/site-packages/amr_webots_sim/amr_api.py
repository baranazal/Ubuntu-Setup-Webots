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
from flask import Flask, jsonify, request, Response
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool, Float64MultiArray
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Global state dictionaries with thread locks
robot_status = {}
status_lock = threading.Lock()

nav_status = {}
nav_lock = threading.Lock()

# Belt status with lock
belt_status = {
    "is_running": False,
    "current_speed": 0.0,
    "last_updated": time.time()
}
belt_lock = threading.Lock()

# Ned arm status with lock
ned_status = {
    "moving": False,
    "joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "gripper_position": 0.0,
    "last_updated": time.time()
}
ned_lock = threading.Lock()

# Navigation tolerance parameters
position_tolerance = 0.01
collision_distance = 0

# Ned arm predefined movements
ned_movements = {
    "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "forward": [0.0, 0.3, -0.3, 0.0, 0.0, 0.0],
    "up": [0.0, 0.7, 0.0, 0.0, 0.0, 0.0],
    "down": [0.0, -0.3, 0.0, 0.0, 0.0, 0.0],
    "left": [-0.785, 0.0, 0.0, 0.0, 0.0, 0.0],
    "right": [0.785, 0.0, 0.0, 0.0, 0.0, 0.0],
    "wave": [0.0, 0.3, 0.0, 0.0, 0.5, 0.0],
    "pick_ready": [0.0, 0.4, -0.7, 0.0, 0.3, 0.0],
    "grab": [0.0, 0.5, -0.7, 0.0, 0.0, 0.0],
    "side_pick": [1.57, 0.3, -0.5, 0.0, 0.3, 0.0],
    "show": [0.0, 0.2, -0.4, 0.0, 0.7, 0.0],
    "reach_high": [0.0, 0.7, 0.2, 0.0, 0.3, 0.0],
    "reach_low": [0.0, 0.1, -1.0, 0.0, 0.3, 0.0],
    "rotate_wrist": [0.0, 0.2, -0.2, 0.0, 0.0, 1.0],
    "diagonal": [0.785, 0.4, -0.5, 0.5, 0.3, 0.785],
    "fold": [0.0, 0.0, -1.3, 0.0, 0.0, 0.0],
    "stretch": [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    "twist": [0.0, 0.2, -0.2, 1.57, 0.0, 0.0],
    "shake": [1.0, 0.2, -0.3, 0.0, 0.5, 1.0],
    "point": [0.0, 0.0, -0.5, 0.0, 1.0, 0.0]
}

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
                status["status"] = f"navigating to {target_desc}"
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
                        status["status"] = f"fine positioning - aligning precisely"
                    else:
                        small_correction = angle_diff * 0.3
                        self.send_velocity_command(robot_name, fine_speed, 0.0, small_correction)
                        status["status"] = f"fine positioning - moving precisely"
                        self.get_logger().debug(f"{robot_name} fine positioning: current=({robot_x:.3f}, {robot_y:.3f}), target=({target_x:.3f}, {target_y:.3f})")
                
                elif abs(angle_diff) > angle_tolerance:
                    # Regular turning phase
                    angular_vel = angular_speed * (angle_diff / abs(angle_diff))
                    self.send_velocity_command(robot_name, 0.0, 0.0, angular_vel)
                    status["status"] = f"turning"
                else:
                    # Regular movement phase
                    min_regular_speed = 0.1
                    regular_adjusted_speed = max(min_regular_speed, min(linear_speed, linear_speed * distance))
                    angular_correction = angle_diff * 0.5
                    self.send_velocity_command(robot_name, regular_adjusted_speed, 0.0, angular_correction)
                    status["status"] = f"moving"

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

class BeltController(Node):
    """ROS2 node that handles Conveyor Belt control"""
    
    def __init__(self):
        super().__init__('belt_api_controller')
        
        # Create publishers for belt control
        self.speed_publisher = self.create_publisher(
            Float64, '/conveyor_belt/set_speed', 10)
            
        self.toggle_publisher = self.create_publisher(
            Bool, '/conveyor_belt/toggle', 10)
            
        # Subscribe to belt status updates
        self.speed_subscription = self.create_subscription(
            Float64, '/conveyor_belt/current_speed', self.speed_callback, 10)
            
        self.state_subscription = self.create_subscription(
            Bool, '/conveyor_belt/is_running', self.state_callback, 10)
        
        # Ensure belt starts in stopped state
        self.initialize_belt_state()
            
        self.get_logger().info("Belt controller API initialized")
    
    def initialize_belt_state(self):
        """Initialize the belt to stopped state on startup"""
        # Set belt speed to 0
        msg_speed = Float64()
        msg_speed.data = 0.0
        self.speed_publisher.publish(msg_speed)
        
        # Set belt to stopped state
        msg_toggle = Bool()
        msg_toggle.data = False
        self.toggle_publisher.publish(msg_toggle)
        
        self.get_logger().info("Belt initialized to stopped state")
    
    def speed_callback(self, msg):
        """Update belt speed status"""
        with belt_lock:
            belt_status["current_speed"] = msg.data
            belt_status["last_updated"] = time.time()
    
    def state_callback(self, msg):
        """Update belt running status"""
        with belt_lock:
            belt_status["is_running"] = msg.data
            belt_status["last_updated"] = time.time()
    
    def set_belt_speed(self, speed):
        """Set the belt speed"""
        try:
            speed_float = float(speed)
            # Allow negative speeds for reverse motion
            msg = Float64()
            msg.data = speed_float
            self.speed_publisher.publish(msg)
            
            with belt_lock:
                belt_status["current_speed"] = speed_float
                belt_status["last_updated"] = time.time()
                if speed_float != 0:
                    belt_status["is_running"] = True
                else:
                    belt_status["is_running"] = False
                
            return True, f"Belt speed set to {speed_float}"
        except ValueError:
            return False, "Invalid speed value"
    
    def toggle_belt(self, running):
        """Toggle the belt on or off"""
        try:
            msg = Bool()
            msg.data = bool(running)
            self.toggle_publisher.publish(msg)
            
            with belt_lock:
                belt_status["is_running"] = bool(running)
                belt_status["last_updated"] = time.time()
                
            return True, f"Belt {'started' if running else 'stopped'}"
        except Exception as e:
            return False, f"Error toggling belt: {str(e)}"

class NedController(Node):
    """ROS2 node that handles Ned arm control"""
    
    def __init__(self):
        super().__init__('ned_api_controller')
        
        # Create publishers for joint and gripper control
        self.joint_commands_publisher = self.create_publisher(
            Float64MultiArray, '/ned/joint_commands', 10)
        
        self.gripper_command_publisher = self.create_publisher(
            Float64MultiArray, '/ned/gripper_command', 10)
        
        # Subscribe to joint states
        self.joint_states_subscription = self.create_subscription(
            JointState, '/ned/joint_states', self.joint_states_callback, 10)
        
        # Movement threshold for detecting when arm is moving
        self.movement_threshold = 0.005
        self.last_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # For sequence control
        self.sequence_timer = None
        self.sequence_running = False
        self.movement_queue = []
        
        self.get_logger().info("Ned arm API controller initialized")
    
    def joint_states_callback(self, msg):
        """Process joint states from the Ned arm"""
        try:
            # Get joint positions (first 6 are arm joints)
            positions = list(msg.position[:6])
            
            # Check if the arm is moving by comparing with last positions
            moving = False
            for i, pos in enumerate(positions):
                if abs(pos - self.last_positions[i]) > self.movement_threshold:
                    moving = True
                    break
            
            # Update last positions
            self.last_positions = positions[:]
            
            # If any gripper position is available (there should be 2)
            gripper_pos = 0.0
            if len(msg.position) > 6:
                # Take the first gripper position and scale it to 0-1 range
                gripper_pos = msg.position[6] / 0.01  # Scale from actual position to 0-1 range
            
            with ned_lock:
                ned_status["joint_positions"] = positions
                ned_status["gripper_position"] = gripper_pos
                ned_status["moving"] = moving
                ned_status["last_updated"] = time.time()
            
        except Exception as e:
            self.get_logger().error(f"Error processing Ned joint states: {str(e)}")
    
    def send_joint_command(self, positions):
        """Send a joint command to the Ned arm"""
        if len(positions) != 6:
            self.get_logger().error(f"Expected 6 joint positions, got {len(positions)}")
            return False
        
        try:
            msg = Float64MultiArray()
            msg.data = positions
            self.joint_commands_publisher.publish(msg)
            
            self.get_logger().info(f"Sent joint command: {positions}")
            return True
        except Exception as e:
            self.get_logger().error(f"Error sending joint command: {str(e)}")
            return False
    
    def send_gripper_command(self, position):
        """Send a gripper command to the Ned arm"""
        try:
            # Ensure position is between 0 and 1
            position = max(0.0, min(1.0, position))
            
            msg = Float64MultiArray()
            msg.data = [position]
            self.gripper_command_publisher.publish(msg)
            
            self.get_logger().info(f"Sent gripper command: {position}")
            return True
        except Exception as e:
            self.get_logger().error(f"Error sending gripper command: {str(e)}")
            return False
    
    def execute_predefined_movement(self, movement_name):
        """Execute a predefined movement"""
        if movement_name not in ned_movements:
            self.get_logger().error(f"Movement '{movement_name}' not found")
            return False
        
        return self.send_joint_command(ned_movements[movement_name])
    
    def start_movement_sequence(self, sequence=None, delay=1.0):
        """Start a sequence of movements with a delay between each"""
        if self.sequence_running:
            self.get_logger().warn("Movement sequence already running")
            return False
        
        # If no sequence is provided, use all available movements
        if sequence is None:
            sequence = list(ned_movements.keys())
        
        # Store the sequence and delay
        self.movement_queue = list(sequence)
        self.sequence_delay = delay
        self.sequence_running = True
        
        # Start a new thread to execute the sequence
        sequence_thread = threading.Thread(target=self._execute_sequence, daemon=True)
        sequence_thread.start()
        
        return True
    
    def stop_movement_sequence(self):
        """Stop the current movement sequence"""
        self.sequence_running = False
        self.movement_queue = []
        return True
    
    def _execute_sequence(self):
        """Execute the movement sequence in a separate thread"""
        try:
            # Start with home position
            self.send_joint_command(ned_movements["home"])
            time.sleep(1.0)
            
            # Keep track of how many movements we've executed
            movements_executed = 0
            
            # Continue until all movements are executed or sequence is stopped
            while self.sequence_running and movements_executed < 20:
                # If queue is empty, refill it
                if not self.movement_queue:
                    self.movement_queue = list(ned_movements.keys())
                
                # Get next movement
                movement = self.movement_queue.pop(0)
                
                # Execute movement
                self.get_logger().info(f"Executing movement {movements_executed+1}/20: {movement}")
                self.execute_predefined_movement(movement)
                
                # Increment counter
                movements_executed += 1
                
                # Wait for next movement
                time.sleep(self.sequence_delay)
            
            # Return to home position at the end
            self.send_joint_command(ned_movements["home"])
            self.sequence_running = False
            
            self.get_logger().info("Movement sequence completed")
        except Exception as e:
            self.get_logger().error(f"Error executing movement sequence: {str(e)}")
            self.sequence_running = False

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
                    
                    if "target_coordinates" in robot_nav and robot_nav["target_coordinates"]:
                        nav_info["target_coordinates"] = robot_nav["target_coordinates"]
            
            velocities = None
            if "velocities" in status:
                velocities = status["velocities"]
            
            return jsonify({
                "robot_id": robot_id,
                "position": status["position"],
                "orientation": status["orientation"],
                "moving": status["moving"],
                "last_updated": status["last_updated"],
                "navigation": nav_info,
                "velocities": velocities
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

@app.route('/api/belt/stop', methods=['POST'])
def stop_belt():
    """Stop the conveyor belt"""
    # Check if belt is already stopped
    with belt_lock:
        if not belt_status["is_running"] and belt_status["current_speed"] == 0.0:
            return jsonify({
                "success": True,
                "message": "Belt is already stopped",
                "status": "stopped"
            })
    
    # Stop the belt
    success, _ = belt_controller.set_belt_speed(0.0)
    
    if success:
        return jsonify({
            "success": True,
            "message": "Belt stopped"
        })
    else:
        return jsonify({"success": False, "error": "Failed to stop belt"}), 500

@app.route('/api/belt/movement', methods=['POST'])
def control_belt_movement():
    """Control belt movement with direction and speed parameters"""
    if not request.json:
        return jsonify({"success": False, "error": "JSON payload required"}), 400
    
    # Check for required parameters
    if 'direction' not in request.json:
        return jsonify({"success": False, "error": "Direction parameter required"}), 400
    if 'speed' not in request.json:
        return jsonify({"success": False, "error": "Speed parameter required"}), 400
    
    direction = request.json['direction']
    speed_value = float(request.json['speed'])
    original_speed = speed_value  # Store original input for reporting
    
    # Validate direction
    if direction not in ['forward', 'reverse']:
        return jsonify({"success": False, "error": "Direction must be 'forward' or 'reverse'"}), 400
    
    # Set speed according to direction - always use absolute value
    abs_speed = abs(speed_value)
    if direction == 'forward':
        target_speed = abs_speed
    else:  # reverse
        target_speed = -abs_speed
    
    # Set the belt speed
    success, _ = belt_controller.set_belt_speed(target_speed)
    
    if success:
        response = {
            "success": True,
            "message": f"Belt moving in {direction} direction at speed {abs_speed}",
            "direction": direction,
            "speed": abs_speed
        }
        # Add note if original speed was negative
        if original_speed < 0:
            response["note"] = f"Input speed was negative ({original_speed}), absolute value ({abs_speed}) was used"
            
        return jsonify(response)
    else:
        return jsonify({"success": False, "error": "Failed to control belt movement"}), 500

@app.route('/api/belt/status', methods=['GET'])
def get_belt_status():
    """Get the current status of the conveyor belt"""
    with belt_lock:
        speed = belt_status["current_speed"]
        direction = "forward" if speed >= 0 else "reverse"
        status = "running" if belt_status["is_running"] else "stopped"
        
        # Create response with fields in specified order
        from collections import OrderedDict
        response_data = OrderedDict([
            ("direction", direction),
            ("status", status),
            ("raw_speed", speed),   # Raw speed value with sign
            ("speed", abs(speed)),  # Absolute speed value
            ("last_updated", belt_status["last_updated"]),
            ("timestamp", time.time())
        ])
        
        # Use json.dumps instead of jsonify to preserve order
        return Response(
            json.dumps(response_data),
            mimetype='application/json'
        )

@app.route('/api/ned/status', methods=['GET'])
def get_ned_status():
    """API endpoint to get the status of the Ned arm"""
    with ned_lock:
        is_moving = ned_status["moving"]
        positions = ned_status["joint_positions"]
        gripper_position = ned_status["gripper_position"]
        last_updated = ned_status["last_updated"]
        
        return jsonify({
            "moving": is_moving,
            "joint_positions": positions,
            "gripper_position": gripper_position,
            "last_updated": last_updated,
            "timestamp": time.time()
        })

@app.route('/api/ned/movements', methods=['GET'])
def get_ned_movements():
    """API endpoint to get available predefined movements"""
    return jsonify({
        "available_movements": list(ned_movements.keys())
    })

@app.route('/api/ned/move', methods=['POST'])
def move_ned_arm():
    """API endpoint to move the Ned arm"""
    global ned_controller
    
    if not request.json:
        return jsonify({"success": False, "error": "JSON payload required"}), 400
    
    # Check for movement name or joint positions
    if 'movement' in request.json:
        # Move to predefined position
        movement_name = request.json['movement']
        if movement_name not in ned_movements:
            return jsonify({
                "success": False, 
                "error": f"Unknown movement: {movement_name}",
                "available_movements": list(ned_movements.keys())
            }), 400
        
        if ned_controller.execute_predefined_movement(movement_name):
            return jsonify({
                "success": True,
                "message": f"Executing movement: {movement_name}",
                "joint_positions": ned_movements[movement_name]
            })
        else:
            return jsonify({
                "success": False,
                "error": "Failed to execute movement"
            }), 500
    
    elif 'joint_positions' in request.json:
        # Move to specific joint positions
        try:
            positions = request.json['joint_positions']
            if len(positions) != 6:
                return jsonify({
                    "success": False,
                    "error": f"Expected 6 joint positions, got {len(positions)}"
                }), 400
            
            positions = [float(p) for p in positions]
            
            if ned_controller.send_joint_command(positions):
                return jsonify({
                    "success": True,
                    "message": "Executing joint command",
                    "joint_positions": positions
                })
            else:
                return jsonify({
                    "success": False,
                    "error": "Failed to send joint command"
                }), 500
        except Exception as e:
            return jsonify({
                "success": False,
                "error": f"Error processing joint positions: {str(e)}"
            }), 400
    
    elif 'gripper_position' in request.json:
        # Control the gripper
        try:
            position = float(request.json['gripper_position'])
            
            if ned_controller.send_gripper_command(position):
                return jsonify({
                    "success": True,
                    "message": f"Setting gripper position to {position}",
                    "gripper_position": position
                })
            else:
                return jsonify({
                    "success": False,
                    "error": "Failed to send gripper command"
                }), 500
        except Exception as e:
            return jsonify({
                "success": False,
                "error": f"Error processing gripper position: {str(e)}"
            }), 400
    
    else:
        return jsonify({
            "success": False,
            "error": "Missing required field: 'movement', 'joint_positions', or 'gripper_position'"
        }), 400

@app.route('/api/ned/sequence', methods=['POST'])
def start_ned_sequence():
    """API endpoint to start a sequence of 20 movements"""
    global ned_controller
    
    # Check if sequence is already running
    if ned_controller.sequence_running:
        return jsonify({
            "success": False,
            "error": "Movement sequence already running"
        }), 400
    
    # Get optional parameters from request
    delay = 1.0  # Default delay between movements
    if request.json and 'delay' in request.json:
        try:
            delay = float(request.json['delay'])
            if delay < 0.5:
                delay = 0.5  # Minimum delay to avoid too rapid movements
            elif delay > 5.0:
                delay = 5.0  # Maximum delay
        except ValueError:
            pass
    
    # Start the sequence
    if ned_controller.start_movement_sequence(delay=delay):
        return jsonify({
            "success": True,
            "message": "Started movement sequence of 20 movements",
            "delay": delay,
            "status_endpoint": "/api/ned/status"
        })
    else:
        return jsonify({
            "success": False,
            "error": "Failed to start movement sequence"
        }), 500

@app.route('/api/ned/sequence/stop', methods=['POST'])
def stop_ned_sequence():
    """API endpoint to stop the current movement sequence"""
    global ned_controller
    
    # Check if sequence is running
    if not ned_controller.sequence_running:
        return jsonify({
            "success": True,
            "message": "No movement sequence is currently running"
        })
    
    # Stop the sequence
    if ned_controller.stop_movement_sequence():
        return jsonify({
            "success": True,
            "message": "Movement sequence stopped"
        })
    else:
        return jsonify({
            "success": False,
            "error": "Failed to stop movement sequence"
        }), 500

def run_flask_server():
    """Run the Flask server"""
    app.run(host='0.0.0.0', port=5000, debug=False)

def main():
    """Main function to start ROS nodes and Flask server"""
    # Initialize ROS2
    rclpy.init()
    
    # Create and initialize ROS nodes
    global navigator_node  # Keep original variable name for compatibility 
    global monitor_node    # Keep original variable name for compatibility
    global belt_controller
    global ned_controller
    
    monitor_node = AMRStatusMonitor()
    navigator_node = AMRNavigator()
    belt_controller = BeltController()
    ned_controller = NedController()
    
    # Create executor for spinning nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(monitor_node)
    executor.add_node(navigator_node)
    executor.add_node(belt_controller)
    executor.add_node(ned_controller)
    
    # Start spinning in separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Start Flask server
    run_flask_server()
    
    # Clean up
    executor.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 