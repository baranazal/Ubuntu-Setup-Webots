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

orchestration_status = {
    "active": False,
    "state": "idle",
    "robot_id": None,
    "step": None,
    "error": None,
    "start_time": None,
    "last_updated": time.time()
}
orchestration_lock = threading.Lock()

PICKUP_POSITIONS = {
    "amr_position": {"x": 0.237, "y": 0.0, "z": 0.0}
    
    }

PICKUP_SEQUENCE = [
    {"action": "move", "movement": "home", "delay": 1.0},
    {"action": "gripper", "position": 1.0, "delay": 0.5},    # Open gripper wide
    
    {"action": "move", "joint_positions": [1.57, 0.0, 0.0, 0.0, 0.0, 0.0], "delay": 2.0},
    
    {"action": "move", "joint_positions": [1.57, 0.0, 0.96, 0.0, 0.0, 0.0], "delay": 1.5},
    
    {"action": "move", "joint_positions": [1.57, 0.08, 0.96, 0.0, 0.0, 0.0], "delay": 2.0},
    
    {"action": "gripper", "position": -1.4, "delay": 1.5},
    
    {"action": "wait", "delay": 1.0},
    
    {"action": "move", "joint_positions": [1.57, 0.0, 0.0, 0.0, 0.0, 0.0], "delay": 1.5},
    
    {"action": "move", "joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "delay": 5},

    {"action": "gripper", "position": 0.8, "delay": 0.5},
    
    {"action": "wait", "delay": 1.0},
    
    {"action": "move", "movement": "home", "delay": 1.0}
]

position_tolerance = 0.01
collision_distance = 0

OPTIMAL_PICKUP_POSITION = {
    "x": 1.0,
    "y": 0.0,
    "z": 0.0
}

ned_movements = {
    "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "forward": [0.0, 0.3, -0.3, 0.0, 0.0, 0.0],
    "up": [0.0, 0.7, 0.0, 0.0, 0.0, 0.0],
    "down": [0.0, -0.3, 0.0, 0.0, 0.0, 0.0],
    "left": [-0.785, 0.0, 0.0, 0.0, 0.0, 0.0],
    "right": [0.785, 0.0, 0.0, 0.0, 0.0, 0.0],
    "wave": [0.0, 0.3, 0.0, 0.0, 0.5, 0.0],
    
    "pre_pickup": [0.0, 0.4, -0.2, 0.0, 0.0, 0.0],
    "approach_box": [0.0, 0.5, -0.4, 0.0, 0.2, 0.0],
    "grasp_position": [0.0, 0.6, -0.6, 0.0, 0.4, 0.0],
    "lift_box": [0.0, 0.5, -0.3, 0.0, 0.4, 0.0],
    "rotate_to_conveyor": [1.57, 0.4, -0.3, 0.0, 0.4, 0.0],
    "position_over_conveyor": [1.57, 0.5, -0.5, 0.0, 0.4, 0.0],
    
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
    "point": [0.0, 0.0, -0.5, 0.0, 1.0, 0.0],
    
    "prepare_for_pickup": [0.0, -0.3, 0.2, 0.0, 0.5, 0.0],
    "approach_amr": [0.0, -0.5, 0.3, 0.0, 0.8, 0.0],
    "pickup_position": [0.0, -0.6, 0.4, 0.0, 0.8, 0.0],
    "lift_from_amr": [0.0, -0.3, 0.1, 0.0, 0.8, 0.0],
    "rotate_to_belt": [1.57, -0.3, 0.1, 0.0, 0.8, 0.0],
    "approach_belt": [1.57, -0.4, 0.2, 0.0, 0.8, 0.0],
    "place_on_belt": [1.57, -0.5, 0.3, 0.0, 0.8, 0.0]
}

orchestration_thread = None
should_stop_orchestration = False

def orchestration_worker(robot_id):
    """Background worker for orchestrating the pickup workflow"""
    global orchestration_status, navigator_node, ned_controller, belt_controller, should_stop_orchestration
    
    try:
        with orchestration_lock:
            orchestration_status["state"] = "starting"
            orchestration_status["step"] = "positioning_amr"
            orchestration_status["last_updated"] = time.time()
        
        amr_pos = PICKUP_POSITIONS["amr_position"]
        print(f"Starting orchestration: Redirecting {robot_id} to position ({amr_pos['x']}, {amr_pos['y']}, {amr_pos['z']})")
        
        if not navigator_node.start_navigation_to_coordinates(robot_id, amr_pos['x'], amr_pos['y'], amr_pos['z']):
            with orchestration_lock:
                orchestration_status["state"] = "error"
                orchestration_status["error"] = "Failed to start navigation"
                orchestration_status["active"] = False
                orchestration_status["last_updated"] = time.time()
            return
        
        position_reached = False
        timeout = time.time() + 30
        
        while time.time() < timeout and not position_reached and not should_stop_orchestration:
            with status_lock:
                if robot_id in robot_status:
                    robot_x = robot_status[robot_id]["position"]["x"]
                    robot_y = robot_status[robot_id]["position"]["y"]
                    
                    distance = math.sqrt((robot_x - amr_pos['x'])**2 + (robot_y - amr_pos['y'])**2)
                    
                    if distance < position_tolerance:
                        position_reached = True
                        with orchestration_lock:
                            orchestration_status["state"] = "amr_positioned"
                            orchestration_status["step"] = "initiating_pickup"
                            orchestration_status["last_updated"] = time.time()
            
            if not position_reached:
                time.sleep(0.5)
        
        if should_stop_orchestration:
            with orchestration_lock:
                orchestration_status["state"] = "cancelled"
                orchestration_status["active"] = False
                orchestration_status["last_updated"] = time.time()
            return
        
        if not position_reached:
            with orchestration_lock:
                orchestration_status["state"] = "error"
                orchestration_status["error"] = "Timeout waiting for AMR to reach position"
                orchestration_status["active"] = False
                orchestration_status["last_updated"] = time.time()
            return
        
        with orchestration_lock:
            orchestration_status["state"] = "executing_pickup"
            orchestration_status["step"] = "starting_arm_sequence"
            orchestration_status["last_updated"] = time.time()
        
        sequence_name = f"pickup_sequence_{robot_id}_{int(time.time())}"
        
        ned_controller.custom_sequences[sequence_name] = PICKUP_SEQUENCE
        
        print(f"Executing pickup sequence: {sequence_name}")
        ned_controller.execute_sequence(sequence_name)
        
        with orchestration_lock:
            orchestration_status["step"] = "waiting_for_sequence_completion"
            orchestration_status["last_updated"] = time.time()
            
        sequence_timeout = time.time() + 30
        while time.time() < sequence_timeout and ned_controller.sequence_running and not should_stop_orchestration:
            time.sleep(0.5)
            
            with orchestration_lock:
                orchestration_status["last_updated"] = time.time()
        
        if should_stop_orchestration:
            with orchestration_lock:
                orchestration_status["state"] = "cancelled"
                orchestration_status["active"] = False
                orchestration_status["last_updated"] = time.time()
            return
        
        with orchestration_lock:
            orchestration_status["state"] = "starting_conveyor"
            orchestration_status["step"] = "belt_movement"
            orchestration_status["last_updated"] = time.time()
        
        belt_controller.set_belt_speed(0.25)
        time.sleep(5.0)
        belt_controller.set_belt_speed(0.0)
        
        if sequence_name in ned_controller.custom_sequences:
            del ned_controller.custom_sequences[sequence_name]
        
        with orchestration_lock:
            orchestration_status["state"] = "completed"
            orchestration_status["active"] = False
            orchestration_status["last_updated"] = time.time()
        
        print(f"Orchestration completed successfully for {robot_id}")
        
    except Exception as e:
        print(f"Error in orchestration: {str(e)}")
        with orchestration_lock:
            orchestration_status["state"] = "error"
            orchestration_status["error"] = str(e)
            orchestration_status["active"] = False
            orchestration_status["last_updated"] = time.time()

class AMRNavigator(Node):
    """ROS2 node that handles AMR navigation commands"""
    
    def __init__(self):
        super().__init__('amr_navigator')
        
        self.cmd_vel_publishers = {}
        
        robot_names = ["AMR1"]
        
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
        """Main navigation control loop"""
        with nav_lock:
            for robot_name, status in nav_status.items():
                if not status["active"]:
                    continue
                
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
                
                dx = target_x - robot_x
                dy = target_y - robot_y
                distance = math.sqrt(dx*dx + dy*dy)
                
                target_angle = math.atan2(dy, dx)
                angle_diff = target_angle - robot_theta
        
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
        
                target_desc = f"coordinates ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})"
                status["status"] = f"navigating to {target_desc}"
                status["last_updated"] = time.time()
                
                if distance < position_tolerance:
                    self.get_logger().info(f"{robot_name} reached target {target_desc}")
                    status["active"] = False
                    status["status"] = "reached"
                    self.stop_robot(robot_name)
                    continue
                
                if not self.is_path_clear(robot_name, target_x, target_y):
                    self.get_logger().warn(f"Path not clear for {robot_name}")
                    status["status"] = "waiting - path blocked"
                    self.stop_robot(robot_name)
                    continue
                
                linear_speed = 0.2
                angular_speed = 0.5
                angle_tolerance = 0.1
                fine_positioning_distance = 0.5
                
                if distance < fine_positioning_distance and distance > position_tolerance:
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
                    angular_vel = angular_speed * (angle_diff / abs(angle_diff))
                    self.send_velocity_command(robot_name, 0.0, 0.0, angular_vel)
                    status["status"] = f"turning"
                else:
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
        robot_names = ["AMR1"]
        
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
        
    def _process_next_movement(self):
        """Process the next movement in the queue"""
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
        elif movement["action"] == "gripper":
            success = self.send_gripper_command(movement["position"])
        elif movement["action"] == "wait":
            success = True
        
        delay = movement.get("delay", 1.0)
        
        if success:
            self.sequence_timer = threading.Timer(delay, self._process_next_movement)
            self.sequence_timer.daemon = True
            self.sequence_timer.start()
        else:
            self.get_logger().error(f"Failed to execute movement: {movement}")
            self.sequence_running = False

class PickupPlaceOperationHandler:
    """Handler for coordinating AMR, Ned arm, and conveyor belt for pickup and place operations"""
    
    def __init__(self, navigator_node, ned_controller, belt_controller):
        self.navigator_node = navigator_node
        self.ned_controller = ned_controller
        self.belt_controller = belt_controller
        self.operation_thread = None
        self.is_running = False
    
    def start_operation(self, amr_id):
        """Start a pickup and place operation with the specified AMR"""
        with pickup_lock:
            if pickup_operation["active"]:
                return False, f"Another operation is active with AMR {pickup_operation['amr_id']}"
            
            with status_lock:
                if amr_id not in robot_status:
                    return False, f"Robot {amr_id} not found"
            
            pickup_operation["active"] = True
            pickup_operation["amr_id"] = amr_id
            pickup_operation["state"] = "starting"
            pickup_operation["start_time"] = time.time()
            pickup_operation["error"] = None
            pickup_operation["last_updated"] = time.time()
        
        if self.operation_thread and self.operation_thread.is_alive():
            self.is_running = False
            self.operation_thread.join(timeout=2.0)
        
        self.is_running = True
        self.operation_thread = threading.Thread(target=self._execute_operation, args=(amr_id,), daemon=True)
        self.operation_thread.start()
        
        return True, f"Pickup and place operation started with AMR {amr_id}"
    
    def _execute_operation(self, amr_id):
        """Execute the pickup and place operation in a separate thread"""
        try:
            self._update_state("moving_amr_to_pickup", amr_id)
            
            success = self.navigator_node.start_navigation_to_coordinates(
                amr_id, 
                OPTIMAL_PICKUP_POSITION["x"], 
                OPTIMAL_PICKUP_POSITION["y"], 
                OPTIMAL_PICKUP_POSITION["z"]
            )
            
            if not success:
                self._update_state("error", amr_id, "Failed to start AMR navigation")
                return
            
            reached_position = False
            timeout = time.time() + 60
            
            while time.time() < timeout and self.is_running:
                with nav_lock:
                    if amr_id in nav_status:
                        if not nav_status[amr_id]["active"] and nav_status[amr_id]["status"] == "reached":
                            reached_position = True
                            break
                
                time.sleep(0.5)
            
            if not reached_position:
                self._update_state("error", amr_id, "AMR did not reach pickup position in time")
                return
            
            self._update_state("positioning_arm", amr_id)
            time.sleep(1.0)
            
            self.ned_controller.send_gripper_command(1.0)
            time.sleep(1.0)
            
            success = self.ned_controller.send_joint_command(ned_movements["prepare_for_pickup"])
            if not success:
                self._update_state("error", amr_id, "Failed to position Ned arm")
                return
            
            time.sleep(2.0)
            
            self._update_state("approaching_box", amr_id)
            success = self.ned_controller.send_joint_command(ned_movements["approach_amr"])
            if not success:
                self._update_state("error", amr_id, "Failed to approach box")
                return
            
            time.sleep(2.0)
            
            self._update_state("grabbing_box", amr_id)
            success = self.ned_controller.send_joint_command(ned_movements["pickup_position"])
            if not success:
                self._update_state("error", amr_id, "Failed to reach pickup position")
                return
            
            time.sleep(2.0)
            
            self.ned_controller.send_gripper_command(0.05)
            time.sleep(1.5)
            
            self._update_state("lifting_box", amr_id)
            success = self.ned_controller.send_joint_command(ned_movements["lift_from_amr"])
            if not success:
                self._update_state("error", amr_id, "Failed to lift box")
                return
            
            time.sleep(2.0)
            
            self._update_state("starting_belt", amr_id)
            self.belt_controller.set_belt_speed(0.25)
            self.belt_controller.toggle_belt(True)
            time.sleep(1.0)
            
            self._update_state("rotating_to_belt", amr_id)
            success = self.ned_controller.send_joint_command(ned_movements["rotate_to_belt"])
            if not success:
                self._update_state("error", amr_id, "Failed to rotate to belt")
                return
            
            time.sleep(2.0)
            
            self._update_state("approaching_belt", amr_id)
            success = self.ned_controller.send_joint_command(ned_movements["approach_belt"])
            if not success:
                self._update_state("error", amr_id, "Failed to approach belt")
                return
            
            time.sleep(2.0)
            
            self._update_state("positioning_over_belt", amr_id)
            success = self.ned_controller.send_joint_command(ned_movements["place_on_belt"])
            if not success:
                self._update_state("error", amr_id, "Failed to position over belt")
                return
            
            time.sleep(2.0)
            
            self._update_state("releasing_box", amr_id)
            self.ned_controller.send_gripper_command(1.0)
            time.sleep(1.5)
            
            self._update_state("returning_home", amr_id)
            success = self.ned_controller.send_joint_command(ned_movements["home"])
            if not success:
                self._update_state("error", amr_id, "Failed to return home")
                return
            
            time.sleep(2.0)
            
            self._update_state("completed", amr_id)
            
        except Exception as e:
            self._update_state("error", amr_id, f"Operation error: {str(e)}")
        
        finally:
            self.is_running = False
    
    def _update_state(self, state, amr_id, error=None):
        """Update the operation state with thread safety"""
        with pickup_lock:
            pickup_operation["state"] = state
            pickup_operation["amr_id"] = amr_id
            pickup_operation["error"] = error
            pickup_operation["last_updated"] = time.time()
            
            if state == "error" or state == "completed":
                pickup_operation["active"] = False
    
    def get_operation_status(self):
        """Get the current operation status"""
        with pickup_lock:
            return {
                "active": pickup_operation["active"],
                "amr_id": pickup_operation["amr_id"],
                "state": pickup_operation["state"],
                "start_time": pickup_operation["start_time"],
                "error": pickup_operation["error"],
                "last_updated": pickup_operation["last_updated"],
                "elapsed_time": time.time() - pickup_operation["start_time"] if pickup_operation["start_time"] else None
            }
    
    def stop_operation(self):
        """Stop the current operation"""
        self.is_running = False
        
        with pickup_lock:
            pickup_operation["active"] = False
            pickup_operation["state"] = "stopped"
            pickup_operation["error"] = "Operation manually stopped"
            pickup_operation["last_updated"] = time.time()
        
        if pickup_operation["amr_id"]:
            self.navigator_node.stop_robot(pickup_operation["amr_id"])
        
        self.ned_controller.send_joint_command(ned_movements["home"])
        
        self.belt_controller.toggle_belt(False)
        
        return True

operation_handler = None

app = Flask(__name__)

############################################################ START OF TESTING API ############################################################  
# This one performs the entire orchestration from start to finish
@app.route('/api/orchestrate-pickup', methods=['POST'])
def orchestrate_pickup():
    """
    Orchestrates the entire pickup workflow:
    1. Position the AMR at the optimal location for the robotic arm
    2. Execute the robotic arm pickup sequence
    3. Start the conveyor belt to transport the box
    
    This is a composite API that coordinates multiple systems.
    """
    global orchestration_thread, should_stop_orchestration, navigator_node, orchestration_status
    
    with orchestration_lock:
        if orchestration_status["active"]:
            current_state = orchestration_status["state"]
            return jsonify({
                "success": False,
                "error": f"Orchestration already in progress (state: {current_state})",
                "orchestration_status": orchestration_status
            }), 409
    
    data = request.json
    if not data:
        return jsonify({"success": False, "error": "Missing request body"}), 400
    
    if 'robot_id' not in data:
        return jsonify({"success": False, "error": "Missing required field: robot_id"}), 400
    
    robot_id = data['robot_id']
    
    with status_lock:
        if robot_id not in robot_status:
            return jsonify({
                "success": False,
                "error": f"Robot {robot_id} not found",
                "available_robots": list(robot_status.keys())
            }), 404
    
    with orchestration_lock:
        orchestration_status["active"] = True
        orchestration_status["state"] = "initializing"
        orchestration_status["robot_id"] = robot_id
        orchestration_status["step"] = "startup"
        orchestration_status["error"] = None
        orchestration_status["start_time"] = time.time()
        orchestration_status["last_updated"] = time.time()
    
    should_stop_orchestration = False
    
    orchestration_thread = threading.Thread(
        target=orchestration_worker,
        args=(robot_id,),
        daemon=True
    )
    orchestration_thread.start()
    
    return jsonify({
        "success": True,
        "message": f"Orchestration started for {robot_id}",
        "robot_id": robot_id,
        "status_endpoint": "/api/orchestrate-pickup/status",
        "cancel_endpoint": "/api/orchestrate-pickup/cancel"
    })
# this one is used to get the status of the orchestration
@app.route('/api/orchestrate-pickup/status', methods=['GET'])
def get_orchestration_status():
    """Get the current status of the pickup orchestration"""
    with orchestration_lock:
        status_copy = dict(orchestration_status)
        status_copy["timestamp"] = time.time()
        
        if status_copy["start_time"]:
            status_copy["elapsed_seconds"] = time.time() - status_copy["start_time"]
        
        return jsonify(status_copy)

@app.route('/api/orchestrate-pickup/cancel', methods=['POST'])
def cancel_orchestration():
    """Cancel the current pickup orchestration"""
    global should_stop_orchestration
    
    with orchestration_lock:
        if not orchestration_status["active"]:
            return jsonify({
                "success": True,
                "message": "No orchestration is currently active"
            })
        
        robot_id = orchestration_status["robot_id"]
    
    should_stop_orchestration = True
    
    if navigator_node and robot_id:
        navigator_node.stop_robot(robot_id)
    
    if ned_controller:
        if ned_controller.sequence_running and ned_controller.sequence_timer:
            ned_controller.sequence_timer.cancel()
        ned_controller.sequence_running = False
        ned_controller.movement_queue = []
        ned_controller.send_joint_command(ned_movements["home"])
    
    if belt_controller:
        belt_controller.set_belt_speed(0.0)
    
    return jsonify({
        "success": True,
        "message": f"Orchestration cancellation requested for {robot_id}",
        "note": "The orchestration will be cancelled soon"
    })
####################################################################### END OF TESTING API #######################################################################

################################################################### START OF INDIVIDUAL COMPONENT APIs ####################################################################
# This one is used to move the AMR to a specified coordinates
@app.route('/api/movement/AMR1', methods=['POST'])
def move_amr1():
    """
    API endpoint to move the AMR1 to specified coordinates
    
    Expected JSON payload:
    {
        "robot_id": "AMR1",  // Robot ID to move
        "x": 1.0,           // Target X coordinate
        "y": 0.0,           // Target Y coordinate
        "z": 0.0            // Target Z coordinate
    }
    """
    global navigator_node
    
    data = request.json
    if not data:
        return jsonify({"success": False, "error": "Missing request body"}), 400
    
    # Validate required fields
    required_fields = ['robot_id', 'x', 'y', 'z']
    for field in required_fields:
        if field not in data:
            return jsonify({"success": False, "error": f"Missing required field: {field}"}), 400
    
    robot_id = data['robot_id']
    target_x = float(data['x'])
    target_y = float(data['y'])
    target_z = float(data['z'])
    
    with status_lock:
        if robot_id not in robot_status:
            available_robots = list(robot_status.keys())
            if len(available_robots) == 0:
                error_msg = "No robots are currently available in the simulation"
            else:
                error_msg = f"Robot '{robot_id}' not found. Currently, only these robots exist: {', '.join(available_robots)}"
                
            return jsonify({
                "success": False,
                "error": error_msg,
                "available_robots": available_robots
            }), 404
            
    if navigator_node and navigator_node.start_navigation_to_coordinates(robot_id, target_x, target_y, target_z):
        return jsonify({
            "success": True,
            "message": f"Robot {robot_id} is being redirected to coordinates ({target_x}, {target_y}, {target_z})",
            "robot_id": robot_id,
            "target_position": {"x": target_x, "y": target_y, "z": target_z}
        })
    else:
        return jsonify({
            "success": False,
            "error": "Failed to start navigation",
            "status": "failed"
        }), 500

@app.route('/api/movement/robotic_arm', methods=['POST'])
def move_robotic_arm():
    """
    API endpoint to control the Ned robotic arm
    
    Expected JSON payload:
    {
        "sequence": [
            {"action": "move", "movement": "home", "delay": 1.0},
            {"action": "gripper", "position": 1.0, "delay": 0.5},
            {"action": "move", "joint_positions": [0.0, 0.3, 0.0, 0.0, 0.0, 0.0], "delay": 2.0},
            // More steps as needed
        ]
    }
    """
    global ned_controller
    
    data = request.json
    if not data:
        return jsonify({"success": False, "error": "Missing request body"}), 400
    
    if 'sequence' not in data:
        return jsonify({
            "success": False, 
            "error": "Missing required field: 'sequence'"
        }), 400
    
    try:
        sequence = data['sequence']
        
        # Validate the sequence
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
        
        # Execute the sequence
        ned_controller.execute_movement_sequence(sequence)
        
        return jsonify({
            "success": True,
            "message": "Executing custom movement sequence",
            "sequence_length": len(sequence)
        })
        
    except Exception as e:
        return jsonify({
            "success": False,
            "error": f"Error executing sequence: {str(e)}"
        }), 400

# This one is used to control the conveyor belt
@app.route('/api/movement/conveyor_belt', methods=['POST'])
def control_conveyor_belt():
    """
    API endpoint to control the conveyor belt
    
    Expected JSON payload:
    {
        "speed": 0.5,            // Speed value (positive number)
        "direction": "forward",  // "forward" or "reverse"
        "running": true          // true to run the belt, false to stop it
    }
    """
    global belt_controller
    
    data = request.json
    if not data:
        return jsonify({"success": False, "error": "Missing request body"}), 400
    
    # Validate required fields
    required_fields = ['speed', 'direction', 'running']
    for field in required_fields:
        if field not in data:
            return jsonify({"success": False, "error": f"Missing required field: {field}"}), 400
    
    try:
        speed_value = float(data['speed'])
        direction = data['direction']
        running = bool(data['running'])
        
        if direction not in ['forward', 'reverse']:
            return jsonify({"success": False, "error": "Direction must be 'forward' or 'reverse'"}), 400
        
        if not running:
            # Stop the belt
            success, _ = belt_controller.set_belt_speed(0.0)
            if success:
                return jsonify({
                    "success": True,
                    "message": "Belt stopped"
                })
            else:
                return jsonify({"success": False, "error": "Failed to stop belt"}), 500
        else:
            # Set the belt speed and direction
            abs_speed = abs(speed_value)
            target_speed = abs_speed if direction == 'forward' else -abs_speed
            
            success, _ = belt_controller.set_belt_speed(target_speed)
            
            if success:
                return jsonify({
                    "success": True,
                    "message": f"Belt moving in {direction} direction at speed {abs_speed}",
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
            available_robots = list(robot_status.keys())
            if len(available_robots) == 0:
                error_msg = "No robots are currently available in the simulation"
            else:
                error_msg = f"Robot '{robot_id}' not found. Currently, only these robots exist: {', '.join(available_robots)}"
            
            return jsonify({
                "error": error_msg,
                "available_robots": available_robots
            }), 404

@app.route('/api/belt/status', methods=['GET'])
def get_belt_status():
    """Get the current status of the conveyor belt"""
    with belt_lock:
        speed = belt_status["current_speed"]
        direction = "forward" if speed >= 0 else "reverse"
        status = "running" if abs(speed) > 0.0 else "stopped"
        
        from collections import OrderedDict
        response_data = OrderedDict([
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

def run_flask_server(host='0.0.0.0', port=5000):
    """Run the Flask server"""
    app.run(host=host, port=port, debug=False)

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
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(monitor_node)
    executor.add_node(navigator_node)
    executor.add_node(belt_controller)
    executor.add_node(ned_controller)
    
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    run_flask_server(host, port)
    
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