#!/usr/bin/env python3

"""
This module provides a Flask API to monitor and control AMR robots, conveyor belts, and robotic arms.
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
import signal
import functools

robot_status = {}
status_lock = threading.Lock()

nav_status = {}
nav_lock = threading.Lock()

belt_status = {
    "is_running": False,
    "current_speed": 0.0,
    "last_updated": time.time()
}
belt_lock = threading.Lock()
belt_controller = None

ned_status = {
    "moving": False,
    "joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "gripper_position": 0.0,
    "last_updated": time.time()
}
ned_lock = threading.Lock()

pickup_operation = {
    "active": False,
    "amr_id": None,
    "state": "idle",
    "start_time": None,
    "error": None,
    "last_updated": time.time()
}
pickup_lock = threading.Lock()

position_tolerance = 0.02
collision_distance = 0

# Robots arrays
agv_names = ["demo_agv001"]
arm_names = ["demo_arm001"]
belt_names = ["demo_belt001"]

ned_movements = {
    "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}

def normalize_angle(angle):
    """Normalize angle to [-pi, pi] range"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def angle_difference(target_angle, current_angle):
    """Calculate the shortest angular difference between two angles"""
    diff = normalize_angle(target_angle - current_angle)
    return diff

class AMRNavigator(Node):
    """ROS2 node that handles AMR navigation commands with orientation support"""
    
    def __init__(self):
        super().__init__('amr_navigator')
        
        self.cmd_vel_publishers = {}
        
        for robot_name in agv_names:
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
                    "target_orientation": None,
                    "status": "idle",
                    "last_updated": time.time(),
                    "settling_start_time": None
                }
            
            self.get_logger().info(f"Navigation control ready for {robot_name}")
        
        self.nav_timer = self.create_timer(0.2, self.navigation_loop)
        
        self.loop_count = 0
        self.last_performance_log = time.time()
    
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
    
    def start_navigation_to_coordinates(self, robot_name, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """Start navigation for a robot to specific coordinates with orientation"""
        with nav_lock:
            if robot_name not in nav_status:
                self.get_logger().error(f"Robot {robot_name} not found for navigation")
                return False
            
            nav_status[robot_name] = {
                "active": True,
                "target_coordinates": {"x": x, "y": y, "z": z},
                "target_orientation": {"roll": roll, "pitch": pitch, "yaw": yaw},
                "status": "starting",
                "last_updated": time.time(),
                "error": None,
                "settling_start_time": None
            }
            
            self.get_logger().info(f"Starting navigation for {robot_name} to coordinates ({x}, {y}, {z}) with orientation (roll: {roll}, pitch: {pitch}, yaw: {yaw})")
            return True
    
    def is_path_clear(self, robot_name, target_x, target_y):
        """Check if there are any robots in the path to the target coordinates"""
        with status_lock:
            if robot_name not in robot_status:
                return False
            
            robot_x = robot_status[robot_name]["position"]["x"]
            robot_y = robot_status[robot_name]["position"]["y"]
            
            dx = target_x - robot_x
            dy = target_y - robot_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > 0:
                dx = dx / distance
                dy = dy / distance
            
            for other_robot, status in robot_status.items():
                if other_robot == robot_name:
                    continue
                
                other_x = status["position"]["x"]
                other_y = status["position"]["y"]
                
                rdx = other_x - robot_x
                rdy = other_y - robot_y
                r_distance = math.sqrt(rdx*rdx + rdy*rdy)
                
                if r_distance < collision_distance:
                    return False
                
                if r_distance > 0:
                    dot = rdx * dx + rdy * dy
                    
                    if 0 < dot < distance:
                        closest_x = robot_x + dx * dot
                        closest_y = robot_y + dy * dot
                        
                        path_distance = math.sqrt((other_x - closest_x)**2 + (other_y - closest_y)**2)
                        
                        if path_distance < collision_distance:
                            return False
            
            return True
    
    def navigation_loop(self):
        """Main navigation control loop with orientation support"""
        self.loop_count += 1
        
        current_time = time.time()
        if current_time - self.last_performance_log > 300:
            self.get_logger().info(f"Navigation loop performance: {self.loop_count} iterations in 5 minutes")
            self.loop_count = 0
            self.last_performance_log = current_time
        
        nav_status_copy = {}
        with nav_lock:
            nav_status_copy = {k: dict(v) for k, v in nav_status.items()}
        
        for robot_name, status in nav_status_copy.items():
            if not status["active"]:
                continue
            
            if "target_coordinates" not in status or not status["target_coordinates"]:
                self.get_logger().error(f"No valid target for {robot_name}")
                with nav_lock:
                    if robot_name in nav_status:
                        nav_status[robot_name]["active"] = False
                        nav_status[robot_name]["status"] = "error"
                        nav_status[robot_name]["error"] = "No valid target"
                self.stop_robot(robot_name)
                continue
            
            target_x = status["target_coordinates"]["x"]
            target_y = status["target_coordinates"]["y"]
            target_z = status["target_coordinates"].get("z", 0.0)
            
            target_orientation = status.get("target_orientation", {"roll": 0.0, "pitch": 0.0, "yaw": 0.0})
            target_yaw = target_orientation.get("yaw", 0.0)
            
            robot_pos = None
            robot_theta = None
            robot_moving = None
            
            with status_lock:
                if robot_name not in robot_status:
                    self.get_logger().error(f"Robot {robot_name} position unknown")
                    with nav_lock:
                        if robot_name in nav_status:
                            nav_status[robot_name]["active"] = False
                            nav_status[robot_name]["status"] = "error"
                            nav_status[robot_name]["error"] = "Robot position unknown"
                    continue
                    
                robot_pos = robot_status[robot_name]["position"]
                robot_theta = robot_status[robot_name]["orientation"]["yaw"]
                robot_moving = robot_status[robot_name]["moving"]
            
            robot_x = robot_pos["x"]
            robot_y = robot_pos["y"]
            
            dx = target_x - robot_x
            dy = target_y - robot_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > position_tolerance:
                with nav_lock:
                    if robot_name in nav_status:
                        nav_status[robot_name]["settling_start_time"] = None
                
                target_angle = math.atan2(dy, dx)
                angle_diff = angle_difference(target_angle, robot_theta)
        
                target_desc = f"coordinates ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})"
                
                with nav_lock:
                    if robot_name in nav_status:
                        nav_status[robot_name]["status"] = f"navigating to {target_desc}"
                        nav_status[robot_name]["last_updated"] = time.time()
                
                if not self.is_path_clear(robot_name, target_x, target_y):
                    self.get_logger().warn(f"Path not clear for {robot_name}")
                    with nav_lock:
                        if robot_name in nav_status:
                            nav_status[robot_name]["status"] = "waiting - path blocked"
                    self.stop_robot(robot_name)
                    continue
                
                max_linear_speed = 0.6
                min_linear_speed = 0.08
                angular_speed = 0.5
                angle_tolerance = 0.01
                
                slowdown_distance = 1.5
                
                if distance > slowdown_distance:
                    current_linear_speed = max_linear_speed
                else:
                    speed_ratio = distance / slowdown_distance
                    speed_ratio = speed_ratio * speed_ratio
                    current_linear_speed = min_linear_speed + (max_linear_speed - min_linear_speed) * speed_ratio
                    current_linear_speed = max(min_linear_speed, current_linear_speed)
                
                fine_positioning_distance = 0.5
                
                if distance < fine_positioning_distance:
                    precision_speed = min_linear_speed + 0.05 * (distance / fine_positioning_distance)
                    
                    if abs(angle_diff) > angle_tolerance / 2:
                        angular_vel = (angular_speed * 0.4) * angle_diff / math.pi
                        self.send_velocity_command(robot_name, 0.0, 0.0, angular_vel)
                        with nav_lock:
                            if robot_name in nav_status:
                                nav_status[robot_name]["status"] = f"fine positioning - aligning (dist: {distance:.2f}m)"
                    else:
                        small_correction = angle_diff * 0.3
                        self.send_velocity_command(robot_name, precision_speed, 0.0, small_correction)
                        with nav_lock:
                            if robot_name in nav_status:
                                nav_status[robot_name]["status"] = f"fine positioning - approaching (dist: {distance:.2f}m)"
                
                elif abs(angle_diff) > angle_tolerance:
                    angular_vel = angular_speed * (angle_diff / abs(angle_diff)) * min(1.0, abs(angle_diff) / 0.5)
                    self.send_velocity_command(robot_name, 0.0, 0.0, angular_vel)
                    with nav_lock:
                        if robot_name in nav_status:
                            nav_status[robot_name]["status"] = f"turning toward target (speed: fast)"
                else:
                    angular_correction = angle_diff * 0.3
                    self.send_velocity_command(robot_name, current_linear_speed, 0.0, angular_correction)
                    with nav_lock:
                        if robot_name in nav_status:
                            speed_desc = "fast" if distance > slowdown_distance else "slowing"
                            nav_status[robot_name]["status"] = f"moving to target ({speed_desc}, dist: {distance:.2f}m, speed: {current_linear_speed:.2f})"
            
            else:
                current_time = time.time()
                settling_time = 2.0
                
                settling_start_time = None
                with nav_lock:
                    if robot_name in nav_status:
                        settling_start_time = nav_status[robot_name]["settling_start_time"]
                        
                        if settling_start_time is None:
                            nav_status[robot_name]["settling_start_time"] = current_time
                            settling_start_time = current_time
                            self.get_logger().info(f"{robot_name} reached position, starting 2-second settling period before yaw adjustment")
                
                if settling_start_time and (current_time - settling_start_time) < settling_time:
                    self.stop_robot(robot_name)
                    remaining_time = settling_time - (current_time - settling_start_time)
                    with nav_lock:
                        if robot_name in nav_status:
                            nav_status[robot_name]["status"] = f"settling at position (waiting {remaining_time:.1f}s before yaw adjustment)"
                            nav_status[robot_name]["last_updated"] = time.time()
                    continue
                
                yaw_diff = angle_difference(target_yaw, robot_theta)
                
                orientation_tolerance = 0.01
                
                if abs(yaw_diff) > orientation_tolerance:
                    base_angular_speed = 0.2
                    
                    if abs(yaw_diff) > 0.5:
                        angular_speed = base_angular_speed
                    else:
                        angular_speed = base_angular_speed * (abs(yaw_diff) / 0.5)
                        angular_speed = max(0.05, angular_speed)
                    
                    angular_vel = angular_speed * (yaw_diff / abs(yaw_diff))
                    self.send_velocity_command(robot_name, 0.0, 0.0, angular_vel)
                    
                    with nav_lock:
                        if robot_name in nav_status:
                            current_yaw_deg = math.degrees(robot_theta)
                            target_yaw_deg = math.degrees(target_yaw)
                            yaw_diff_deg = math.degrees(yaw_diff)
                            nav_status[robot_name]["status"] = f"adjusting orientation (current: {current_yaw_deg:.1f}°, target: {target_yaw_deg:.1f}°, diff: {yaw_diff_deg:.1f}°)"
                            nav_status[robot_name]["last_updated"] = time.time()
                else:
                    self.get_logger().info(f"{robot_name} reached target position and orientation (tolerance: {math.degrees(orientation_tolerance):.1f}°)")
                    self.stop_robot(robot_name)
                    
                    time.sleep(0.1)
                    
                    with nav_lock:
                        if robot_name in nav_status:
                            nav_status[robot_name]["active"] = False
                            nav_status[robot_name]["status"] = "reached"
                            final_yaw_deg = math.degrees(robot_theta)
                            target_yaw_deg = math.degrees(target_yaw)
                            nav_status[robot_name]["final_position"] = {
                                "x": robot_x,
                                "y": robot_y,
                                "yaw_degrees": final_yaw_deg,
                                "target_yaw_degrees": target_yaw_deg,
                                "yaw_error_degrees": math.degrees(abs(yaw_diff))
                            }

class AMRStatusMonitor(Node):
    """ROS2 node that subscribes to AMR odometry topics and stores their status with full orientation"""
    
    def __init__(self):
        super().__init__('amr_status_monitor')
        
        self.odom_subscribers = {}
        
        for robot_name in agv_names:
            with status_lock:
                robot_status[robot_name] = {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
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
        """Process odometry data for a specific robot with full orientation"""
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        pos_z = msg.pose.pose.position.z
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
            
        linear_speed = math.sqrt(
            msg.twist.twist.linear.x**2 + 
            msg.twist.twist.linear.y**2
        )
        angular_speed = abs(msg.twist.twist.angular.z)
        is_moving = linear_speed > 0.01 or angular_speed > 0.01
        
        with status_lock:
            robot_status[robot_name] = {
                "position": {"x": pos_x, "y": pos_y, "z": pos_z},
                "orientation": {"roll": roll, "pitch": pitch, "yaw": yaw},
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
        
        self.speed_publisher = self.create_publisher(
            Float64, '/conveyor_belt/set_speed', 10)
            
        self.toggle_publisher = self.create_publisher(
            Bool, '/conveyor_belt/toggle', 10)
            
        self.speed_subscription = self.create_subscription(
            Float64, '/conveyor_belt/current_speed', self.speed_callback, 10)
            
        self.state_subscription = self.create_subscription(
            Bool, '/conveyor_belt/is_running', self.state_callback, 10)
        
        self.initialize_belt_state()
            
        self.get_logger().info("Belt controller API initialized")
    
    def initialize_belt_state(self):
        """Initialize the belt to stopped state on startup"""
        msg_speed = Float64()
        msg_speed.data = 0.0
        self.speed_publisher.publish(msg_speed)
        
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
            msg = Float64()
            msg.data = speed_float
            self.speed_publisher.publish(msg)
            
            with belt_lock:
                belt_status["current_speed"] = speed_float
                belt_status["last_updated"] = time.time()
                belt_status["is_running"] = speed_float != 0.0
                
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
        
        self.joint_commands_publisher = self.create_publisher(
            Float64MultiArray, '/ned/joint_commands', 10)
        
        self.gripper_command_publisher = self.create_publisher(
            Float64MultiArray, '/ned/gripper_command', 10)
        
        self.joint_states_subscription = self.create_subscription(
            JointState, '/ned/joint_states', self.joint_states_callback, 10)
        
        self.movement_threshold = 0.005
        self.last_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.sequence_timer = None
        self.sequence_running = False
        self.movement_queue = []
        self.custom_sequences = {}
        
        self.active_timers = set()
        self.cleanup_interval = 60
        self.last_cleanup = time.time()
        
        self.get_logger().info("Ned arm API controller initialized")
    
    def joint_states_callback(self, msg):
        """Process joint states from the Ned arm"""
        try:
            positions = list(msg.position[:6])
            
            moving = False
            for i, pos in enumerate(positions):
                if abs(pos - self.last_positions[i]) > self.movement_threshold:
                    moving = True
                    break
            
            self.last_positions = positions[:]
            
            gripper_pos = 0.0
            if len(msg.position) > 6:
                gripper_pos = msg.position[6] / 0.01
            
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
    
    def execute_sequence(self, sequence_name):
        """Execute a named sequence of movements"""
        if sequence_name not in self.custom_sequences:
            self.get_logger().error(f"Sequence '{sequence_name}' not found")
            return False
            
        sequence = self.custom_sequences[sequence_name]
        self.execute_movement_sequence(sequence)
        return True
    
    def execute_movement_sequence(self, sequence):
        """Execute a sequence of movements"""
        if self.sequence_running:
            self.get_logger().warning("A sequence is already running, cancelling previous sequence")
            if self.sequence_timer:
                self.sequence_timer.cancel()
                
        self.sequence_running = True
        self.movement_queue = sequence.copy()
        self._process_next_movement()
        
    def _cleanup_resources(self):
        """Clean up stale timers and resources"""
        current_time = time.time()
        if current_time - self.last_cleanup > self.cleanup_interval:
            stale_timers = []
            for timer in self.active_timers:
                if hasattr(timer, 'finished') and timer.finished:
                    stale_timers.append(timer)
            
            for timer in stale_timers:
                self.active_timers.discard(timer)
            
            self.last_cleanup = current_time
            
            if len(self.active_timers) > 10:
                self.get_logger().warning(f"High number of active timers: {len(self.active_timers)}")
    
    def _process_next_movement(self):
        """Process the next movement in the queue"""
        self._cleanup_resources()
        
        if not self.movement_queue:
            self.sequence_running = False
            self.get_logger().info("Movement sequence completed")
            return
            
        movement = self.movement_queue.pop(0)
        success = False
        
        if movement["action"] == "move":
            if "movement" in movement:
                success = self.execute_predefined_movement(movement["movement"])
            elif "joint_positions" in movement:
                success = self.send_joint_command(movement["joint_positions"])
            else:
                self.get_logger().error(f"Move action requires 'movement' or 'joint_positions': {movement}")
        elif movement["action"] == "gripper":
            success = self.send_gripper_command(movement["position"])
        elif movement["action"] == "wait":
            success = True
        
        delay = movement.get("delay", 1.0)
        
        if success:
            if self.sequence_timer:
                self.sequence_timer.cancel()
                self.active_timers.discard(self.sequence_timer)
            
            self.sequence_timer = threading.Timer(delay, self._process_next_movement)
            self.sequence_timer.daemon = True
            self.active_timers.add(self.sequence_timer)
            self.sequence_timer.start()
        else:
            self.get_logger().error(f"Failed to execute movement: {movement}")
            self.sequence_running = False
    
    def stop_sequence(self):
        """Stop current sequence and clean up resources"""
        self.sequence_running = False
        self.movement_queue.clear()
        
        if self.sequence_timer:
            self.sequence_timer.cancel()
            self.active_timers.discard(self.sequence_timer)
            self.sequence_timer = None
        
        for timer in list(self.active_timers):
            timer.cancel()
        self.active_timers.clear()

operation_handler = None

app = Flask(__name__)

################################################################### START OF INDIVIDUAL COMPONENT APIs ####################################################################
# This one is used to move the AGV to a specified coordinates
@app.route('/api/movement/AGV', methods=['POST'])
def move_agv():
    """API endpoint to move the AGV to specified coordinates with orientation"""
    global navigator_node
    
    try:
        data = request.json
        if not data:
            return jsonify({"success": False, "error": "Missing request body"}), 400
        
        required_fields = ['agv_id', 'x', 'y', 'z', 'roll', 'pitch', 'yaw']
        for field in required_fields:
            if field not in data:
                return jsonify({"success": False, "error": f"Missing required field: {field}"}), 400
        
        agv_id = data['agv_id']
        
        if agv_id not in agv_names:
            return jsonify({
                "success": False,
                "error": f"Invalid agv_id '{agv_id}'. Available AGVs: {', '.join(agv_names)}"
            }), 400
        
        target_x = float(data['x'])
        target_y = float(data['y'])
        target_z = float(data['z'])
        
        target_roll = float(data['roll'])
        target_pitch = float(data['pitch'])
        target_yaw = float(data['yaw'])
        
        with status_lock:
            if agv_id not in robot_status:
                available_robots = list(robot_status.keys())
                if len(available_robots) == 0:
                    error_msg = "No AGVs are currently available in the simulation"
                else:
                    error_msg = f"AGV '{agv_id}' not found. Currently, only these AGVs exist: {', '.join(available_robots)}"
                    
                return jsonify({
                    "success": False,
                    "error": error_msg,
                    "available_agvs": available_robots
                }), 404
                
        if navigator_node and navigator_node.start_navigation_to_coordinates(
            agv_id, target_x, target_y, target_z, target_roll, target_pitch, target_yaw
        ):
            return jsonify({
                "success": True,
                "message": f"AGV {agv_id} is being redirected to coordinates ({target_x}, {target_y}, {target_z}) with orientation (roll: {target_roll}, pitch: {target_pitch}, yaw: {target_yaw})",
                "agv_id": agv_id,
                "target_position": {"x": target_x, "y": target_y, "z": target_z},
                "target_orientation": {"roll": target_roll, "pitch": target_pitch, "yaw": target_yaw}
            })
        else:
            return jsonify({
                "success": False,
                "error": "Failed to start navigation",
                "status": "failed"
            }), 500
            
    except ValueError as e:
        return jsonify({
            "success": False,
            "error": f"Invalid coordinate or orientation values: {str(e)}"
        }), 400
    except Exception as e:
        return jsonify({
            "success": False,
            "error": f"Internal server error: {str(e)}"
        }), 500

@app.route('/api/movement/ROBOTIC_ARM', methods=['POST'])
def move_robotic_arm():
    """
    API endpoint to control the Ned robotic arm
    
    Expected JSON payload:
    {
        "arm_id": "arm001",      // ID of the robotic arm (required)
        "sequence": [
            {"action": "move", "joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "delay": 1.0},
            {"action": "gripper", "position": 1.0, "delay": 0.5},
            {"action": "move", "joint_positions": [0.0, 0.3, 0.0, 0.0, 0.0, 0.0], "delay": 2.0},
            {"action": "wait", "delay": 1.0}
        ]
    }
    """
    global ned_controller
    
    data = request.json
    if not data:
        return jsonify({"success": False, "error": "Missing request body"}), 400
    
    required_fields = ['arm_id', 'sequence']
    for field in required_fields:
        if field not in data:
            return jsonify({"success": False, "error": f"Missing required field: {field}"}), 400
    
    arm_id = data['arm_id']
    
    if arm_id not in arm_names:
        return jsonify({
            "success": False,
            "error": f"Invalid arm_id '{arm_id}'. Available arms: {', '.join(arm_names)}"
        }), 400
    
    try:
        sequence = data['sequence']
        
        for step in sequence:
            if 'action' not in step:
                return jsonify({
                    "success": False,
                    "error": f"Missing 'action' in step: {step}"
                }), 400
                
            if step['action'] == 'move':
                if 'movement' not in step and 'joint_positions' not in step:
                    return jsonify({
                        "success": False,
                        "error": f"Move action requires 'movement' or 'joint_positions': {step}"
                    }), 400
                    
                if 'movement' in step and step['movement'] not in ned_movements:
                    return jsonify({
                        "success": False,
                        "error": f"Unknown movement: {step['movement']}",
                        "available_movements": list(ned_movements.keys())
                    }), 400
                    
                if 'joint_positions' in step and len(step['joint_positions']) != 6:
                    return jsonify({
                        "success": False,
                        "error": f"Expected 6 joint positions, got {len(step['joint_positions'])}"
                    }), 400
                    
            elif step['action'] == 'gripper':
                if 'position' not in step:
                    return jsonify({
                        "success": False,
                        "error": f"Gripper action requires 'position': {step}"
                    }), 400
        
        ned_controller.execute_movement_sequence(sequence)
        
        return jsonify({
            "success": True,
            "message": "Executing custom movement sequence",
            "arm_id": arm_id,
            "sequence_length": len(sequence)
        })
        
    except Exception as e:
        return jsonify({
            "success": False,
            "error": f"Error executing sequence: {str(e)}"
        }), 400

# This one is used to control the conveyor belt
@app.route('/api/movement/CONVEYOR_BELT', methods=['POST'])
def control_conveyor_belt():
    """
    API endpoint to control the conveyor belt
    
    Expected JSON payload:
    {
        "belt_id": "belt001",    // ID of the belt (required)
        "speed": 0.5,            // Speed value (positive number)
        "direction": "forward",  // "forward" or "reverse"
        "running": true          // true to run the belt, false to stop it
    }
    """
    global belt_controller
    
    data = request.json
    if not data:
        return jsonify({"success": False, "error": "Missing request body"}), 400
    
    required_fields = ['belt_id', 'speed', 'direction', 'running']
    for field in required_fields:
        if field not in data:
            return jsonify({"success": False, "error": f"Missing required field: {field}"}), 400
    
    belt_id = data['belt_id']
    
    if belt_id not in belt_names:
        return jsonify({
            "success": False,
            "error": f"Invalid belt_id '{belt_id}'. Available belts: {', '.join(belt_names)}"
        }), 400
    
    try:
        speed_value = float(data['speed'])
        direction = data['direction']
        running = bool(data['running'])
        
        if direction not in ['forward', 'reverse']:
            return jsonify({"success": False, "error": "Direction must be 'forward' or 'reverse'"}), 400
        
        if not running:
            success, _ = belt_controller.set_belt_speed(0.0)
            if success:
                return jsonify({
                    "success": True,
                    "message": "Belt stopped",
                    "belt_id": belt_id
                })
            else:
                return jsonify({"success": False, "error": "Failed to stop belt"}), 500
        else:
            abs_speed = abs(speed_value)
            target_speed = abs_speed if direction == 'forward' else -abs_speed
            
            success, _ = belt_controller.set_belt_speed(target_speed)
            
            if success:
                return jsonify({
                    "success": True,
                    "message": f"Belt moving in {direction} direction at speed {abs_speed}",
                    "belt_id": belt_id,
                    "direction": direction,
                    "speed": abs_speed,
                    "running": running
                })
            else:
                return jsonify({"success": False, "error": "Failed to control belt movement"}), 500
    except ValueError:
        return jsonify({"success": False, "error": "Invalid speed value"}), 400

@app.route('/api/status/<robot_id>', methods=['GET'])
def get_status(robot_id):
    """API endpoint to get the status of robots including AGV (agv001), arm (arm001), and belt (belt001)"""
    try:
        if robot_id in agv_names:
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
                            
                            if "target_orientation" in robot_nav and robot_nav["target_orientation"]:
                                nav_info["target_orientation"] = robot_nav["target_orientation"]
                            
                            if "final_position" in robot_nav:
                                nav_info["final_position"] = robot_nav["final_position"]
                    
                    velocities = None
                    if "velocities" in status:
                        velocities = status["velocities"]
                    
                    return jsonify({
                        "robot_id": robot_id,
                        "robot_type": "AGV",
                        "position": status["position"],
                        "orientation": status["orientation"],
                        "moving": status["moving"],
                        "last_updated": status["last_updated"],
                        "navigation": nav_info,
                        "velocities": velocities,
                        "timestamp": time.time()
                    })
                else:
                    available_robots = list(robot_status.keys())
                    if len(available_robots) == 0:
                        error_msg = "No AGV robots are currently available in the simulation"
                    else:
                        error_msg = f"AGV '{robot_id}' not found. Currently, only these robots exist: {', '.join(available_robots)}"
                    
                    return jsonify({
                        "error": error_msg,
                        "available_robots": available_robots
                    }), 404
        
        elif robot_id in arm_names:
            with ned_lock:
                is_moving = ned_status["moving"]
                positions = ned_status["joint_positions"]
                gripper_position = ned_status["gripper_position"]
                last_updated = ned_status["last_updated"]
                
                return jsonify({
                    "robot_id": robot_id,
                    "robot_type": "ROBOTIC_ARM", 
                    "moving": is_moving,
                    "joint_positions": positions,
                    "gripper_position": gripper_position,
                    "last_updated": last_updated,
                    "timestamp": time.time()
                })
        
        elif robot_id in belt_names:
            with belt_lock:
                speed = belt_status["current_speed"]
                direction = "forward" if speed >= 0 else "reverse"
                status = "running" if abs(speed) > 0.0 else "stopped"
                
                from collections import OrderedDict
                response_data = OrderedDict([
                    ("robot_id", robot_id),
                    ("robot_type", "CONVEYOR_BELT"),
                    ("direction", direction),
                    ("status", status),
                    ("raw_speed", speed),
                    ("speed", abs(speed)),
                    ("last_updated", belt_status["last_updated"]),
                    ("timestamp", time.time())
                ])
                
                return Response(
                    json.dumps(response_data),
                    mimetype='application/json'
                )
        
        else:
            return jsonify({
                "error": f"Unknown robot ID '{robot_id}'. Available robots: AGV: {', '.join(agv_names)}, ARM: {', '.join(arm_names)}, BELT: {', '.join(belt_names)}",
                "robot_id": robot_id,
                "available_agv": agv_names,
                "available_arm": arm_names,
                "available_belt": belt_names
            }), 404
                
    except Exception as e:
        return jsonify({
            "success": False,
            "error": f"Internal server error: {str(e)}"
        }), 500

def run_flask_server(host='0.0.0.0', port=5000):
    """Run the Flask server with optimized settings"""
    app.config['JSONIFY_PRETTYPRINT_REGULAR'] = False
    app.config['JSON_SORT_KEYS'] = False
    
    try:
        app.run(
            host=host, 
            port=port, 
            debug=False, 
            threaded=True,
            processes=1,
            use_reloader=False
        )
    except Exception as e:
        print(f"Flask server error: {e}")
        raise

def main(host='0.0.0.0', port=5000):
    """Main function to start ROS nodes and Flask server"""
    rclpy.init()
    
    global navigator_node
    global monitor_node
    global belt_controller
    global ned_controller
    
    monitor_node = AMRStatusMonitor()
    navigator_node = AMRNavigator()
    belt_controller = BeltController()
    ned_controller = NedController()
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(monitor_node)
    executor.add_node(navigator_node)
    executor.add_node(belt_controller)
    executor.add_node(ned_controller)
    
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    def monitor_resources():
        """Monitor system resources and log warnings"""
        import threading
        import gc
        
        while True:
            time.sleep(300)
            
            thread_count = threading.active_count()
            if thread_count > 20:
                print(f"WARNING: High thread count: {thread_count}")
            
            gc.collect()
            
            if ned_controller and hasattr(ned_controller, 'active_timers'):
                timer_count = len(ned_controller.active_timers)
                if timer_count > 5:
                    print(f"WARNING: High timer count in NED controller: {timer_count}")
    
    monitor_thread = threading.Thread(target=monitor_resources, daemon=True)
    monitor_thread.start()
    
    try:
        run_flask_server(host, port)
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Server error: {e}")
    finally:
        if ned_controller and hasattr(ned_controller, 'stop_sequence'):
            ned_controller.stop_sequence()
        
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='AMR API Server')
    parser.add_argument('--host', default='0.0.0.0', 
                        help='Host address to bind to (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=5000, 
                        help='Port to listen on (default: 5000)')
    
    args = parser.parse_args()
    
    main(host=args.host, port=args.port) 