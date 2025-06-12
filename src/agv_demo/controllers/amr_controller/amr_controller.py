#!/usr/bin/env python3

"""
AMR Controller for Webots
This controller manages AMR robots in the Webots simulation,
interfacing with ROS2 topics for each robot's velocity commands.
"""

import sys
import time
import os
from controller import Robot, Motor, GPS, InertialUnit, Supervisor
import socket
import struct
import threading
import math
from datetime import datetime

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, TransformStamped, Point
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Header
    from tf2_ros import TransformBroadcaster
    from builtin_interfaces.msg import Time
    HAS_ROS2 = True
except ImportError:
    print("WARNING: ROS2 python package not found. Running in standalone mode.")
    HAS_ROS2 = False

class AMRController:
    """Controller for AMR robots with mecanum wheels"""
    
    def __init__(self, robot, name=""):
        """Initialize the controller"""
        self.robot = robot
        self.name = name if name else os.environ.get('WEBOTS_ROBOT_NAME', 'AMR')
        
        self.timestep = int(robot.getBasicTimeStep())
        self.robot_path = os.path.dirname(os.path.abspath(__file__))
        
        print(f"Initializing AMR controller for robot {self.name}")
        
        self._init_motors()
        self._init_position_sensors()
        
        # Position and movement variables
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_z = 0.0
        self.prev_theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.last_pose_time = robot.getTime()
        self.sensor_readings_ready = False
        self.debug_counter = 0
        
        self._init_ros()
        
        print(f"Initial position for {self.name}: ({self.x}, {self.y}, {self.z}, {self.theta})")
        
        self.running = True
        
        # Robot parameters
        self.wheel_radius = 0.055  # meters
        self.wheel_distance_x = 0.34  # meters (distance between left and right wheels)
        self.wheel_distance_y = 0.30  # meters (distance between front and rear wheels)
        self.max_wheel_speed = 15.0  # rad/s
        
        self.last_cmd_time = robot.getTime()
        self.cmd_timeout = 0.5  # seconds
    
    def _init_motors(self):
        """Initialize motors with mecanum wheel setup"""
        self.motors = {
            'front_left': self.robot.getDevice('front_left_motor'),
            'front_right': self.robot.getDevice('front_right_motor'),
            'rear_left': self.robot.getDevice('rear_left_motor'),
            'rear_right': self.robot.getDevice('rear_right_motor')
        }
        
        for motor_name, motor in self.motors.items():
            if motor:
                motor.setPosition(float('inf'))  # Set position to infinity for velocity control
                motor.setVelocity(0.0)  # Stop the motor initially
        
        print(f"Available devices for {self.name}:")
        for i in range(self.robot.getNumberOfDevices()):
            device = self.robot.getDeviceByIndex(i)
            if device:
                print(f"  {device.getName()} (Type: {device.getNodeType()})")
    
    def _init_position_sensors(self):
        """Initialize position sensors and related devices"""
        self.has_position_sensors = False
        
        self.wheel_sensors = {
            'front_left': self.robot.getDevice('front_left_sensor'),
            'front_right': self.robot.getDevice('front_right_sensor'),
            'rear_left': self.robot.getDevice('rear_left_sensor'),
            'rear_right': self.robot.getDevice('rear_right_sensor')
        }
        
        for sensor_name, sensor in self.wheel_sensors.items():
            if sensor:
                sensor.enable(self.timestep)
        
        self.gps = self.robot.getDevice('gps') if self.robot.getDevice('gps') else None
        self.imu = self.robot.getDevice('inertial_unit') if self.robot.getDevice('inertial_unit') else None
        
        if self.gps:
            sampling_period = 32  # milliseconds
            self.gps.enable(sampling_period)
            print(f"GPS enabled for {self.name} with sampling period {sampling_period}ms")
            self.has_position_sensors = True
        else:
            print(f"Warning: GPS not found for {self.name}")
        
        if self.imu:
            sampling_period = 32  # milliseconds
            self.imu.enable(sampling_period)
            print(f"IMU enabled for {self.name} with sampling period {sampling_period}ms")
            self.has_position_sensors = True
        else:
            print(f"Warning: IMU not found for {self.name}")
    
    def _init_ros(self):
        """Initialize ROS node and interfaces"""
        try:
            rclpy.init(args=None)
            self.node = rclpy.create_node(f'{self.name}_driver')
            
            self.cmd_vel_sub = self.node.create_subscription(
                Twist,
                f'/{self.name}/cmd_vel',
                self.cmd_vel_callback,
                10
            )
            
            self.odom_pub = self.node.create_publisher(
                Odometry,
                f'/{self.name}/odom',
                10
            )
            
            self.tf_broadcaster = TransformBroadcaster(self.node)
            
            self.odom_timer = self.node.create_timer(0.1, self.publish_odometry)
            
            print(f"ROS2 node initialized for {self.name}")
        except Exception as e:
            print(f"Failed to initialize ROS: {str(e)}")
            self.node = None
    
    def set_velocity(self, vx, vy, omega):
        """Set robot velocity using mecanum wheel kinematics"""
        try:
            self.vx = vx
            self.vy = vy
            self.omega = omega
            
            # Calculate wheel speeds using mecanum kinematic model
            # For a mecanum wheel robot:
            # [ v1 ]   [ 1  1  (lx+ly) ] [ vx  ]
            # [ v2 ] = [ 1 -1 -(lx+ly) ] [ vy  ]
            # [ v3 ]   [ 1 -1  (lx+ly) ] [ omega ]
            # [ v4 ]   [ 1  1 -(lx+ly) ]
            
            lx = self.wheel_distance_x / 2.0
            ly = self.wheel_distance_y / 2.0
            
            front_left = (vx - vy - (lx + ly) * omega) / self.wheel_radius
            front_right = (vx + vy + (lx + ly) * omega) / self.wheel_radius
            rear_left = (vx + vy - (lx + ly) * omega) / self.wheel_radius
            rear_right = (vx - vy + (lx + ly) * omega) / self.wheel_radius
            
            # Limit wheel speeds to max value
            wheel_speeds = [front_left, front_right, rear_left, rear_right]
            max_speed = max(abs(speed) for speed in wheel_speeds)
            
            if max_speed > self.max_wheel_speed:
                scale = self.max_wheel_speed / max_speed
                front_left *= scale
                front_right *= scale
                rear_left *= scale
                rear_right *= scale
            
            if self.motors['front_left']:
                self.motors['front_left'].setVelocity(front_left)
            if self.motors['front_right']:
                self.motors['front_right'].setVelocity(front_right)
            if self.motors['rear_left']:
                self.motors['rear_left'].setVelocity(rear_left)
            if self.motors['rear_right']:
                self.motors['rear_right'].setVelocity(rear_right)
                
            return True
        except Exception as e:
            print(f"ERROR: Failed to update wheel speeds for robot {self.name}: {str(e)}")
            return False
    
    def update_position(self):
        """Update the robot's position using GPS and IMU sensors"""
        current_time = self.robot.getTime()
        dt = current_time - self.last_pose_time
        
        if dt <= 0:
            return False
        
        try:
            # Store previous position for velocity calculation
            self.prev_x = self.x
            self.prev_y = self.y
            self.prev_z = self.z
            self.prev_theta = self.theta
            
            self.debug_counter += 1
            force_debug = self.debug_counter < 10
            
            if self.has_position_sensors:
                # Get position from GPS
                if self.gps and self.gps.getSamplingPeriod() > 0:
                    gps_values = self.gps.getValues()
                    if gps_values and len(gps_values) >= 3:
                        new_x = gps_values[0]
                        new_y = gps_values[1]
                        new_z = gps_values[2]
                        
                        if abs(new_x - self.x) > 0.001 or abs(new_y - self.y) > 0.001 or abs(new_z - self.z) > 0.001 or force_debug:
                            print(f"GPS update for {self.name}: ({new_x:.3f}, {new_y:.3f}, {new_z:.3f}), sampling period: {self.gps.getSamplingPeriod()}")
                        
                        self.x = new_x
                        self.y = new_y
                        self.z = new_z
                    else:
                        print(f"Warning: GPS returned invalid data for {self.name}: {gps_values}")
                else:
                    if force_debug:
                        print(f"Warning: GPS not ready for {self.name}, sampling period: {self.gps.getSamplingPeriod() if self.gps else 'N/A'}")
                        
                        if self.debug_counter == 5:
                            self._get_position_from_world_file()
                
                # Get orientation from IMU
                if self.imu and self.imu.getSamplingPeriod() > 0:
                    imu_values = self.imu.getRollPitchYaw()
                    if imu_values and len(imu_values) >= 3:
                        new_theta = imu_values[2]
                        
                        if abs(new_theta - self.theta) > 0.01 or force_debug:
                            print(f"IMU update for {self.name}: yaw={new_theta:.3f}, sampling period: {self.imu.getSamplingPeriod()}")
                        
                        self.theta = new_theta
                    else:
                        print(f"Warning: IMU returned invalid data for {self.name}: {imu_values}")
                else:
                    if force_debug:
                        print(f"Warning: IMU not ready for {self.name}, sampling period: {self.imu.getSamplingPeriod() if self.imu else 'N/A'}")
                
                # Mark that we have sensor readings now
                if not self.sensor_readings_ready and self.x != 0.0 and self.y != 0.0:
                    self.sensor_readings_ready = True
                    print(f"Initial position for {self.name}: ({self.x:.2f}, {self.y:.2f}, {self.z:.2f}, {self.theta:.2f})")
            else:
                # Use dead reckoning when no sensors are available
                self.x += (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
                self.y += (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
                # Z remains unchanged in dead reckoning as we have no vertical movement
                self.theta += self.omega * dt
                
                # Normalize theta to [-pi, pi]
                while self.theta > math.pi:
                    self.theta -= 2 * math.pi
                while self.theta < -math.pi:
                    self.theta += 2 * math.pi
            
            self.last_pose_time = current_time
            return True
        except Exception as e:
            print(f"ERROR: Failed to update position for robot {self.name}: {str(e)}")
            return False
    
    def _get_position_from_world_file(self):
        """Attempt to get initial position from the world file"""
        try:
            # Use the robot number to determine its position from the world file
            if self.name == "AMR":
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0
                self.theta = 0.0
                print(f"Setting {self.name} position from world file: (0.0, 0.0, 0.0, 0.0)")
            elif self.name == "AMR2":
                self.x = 1.0
                self.y = 0.0
                self.z = 0.0
                self.theta = 0.0
                print(f"Setting {self.name} position from world file: (1.0, 0.0, 0.0, 0.0)")
            elif self.name == "AMR3":
                self.x = 2.0
                self.y = 0.0
                self.z = 0.0
                self.theta = 0.0
                print(f"Setting {self.name} position from world file: (2.0, 0.0, 0.0, 0.0)")
            elif self.name == "AMR4":
                self.x = -1.0
                self.y = 0.0
                self.z = 0.0
                self.theta = 0.0
                print(f"Setting {self.name} position from world file: (-1.0, 0.0, 0.0, 0.0)")
            elif self.name == "AMR5":
                self.x = -2.0
                self.y = 0.0
                self.z = 0.0
                self.theta = 0.0
                print(f"Setting {self.name} position from world file: (-2.0, 0.0, 0.0, 0.0)")
            
            # Update previous positions as well
            self.prev_x = self.x
            self.prev_y = self.y
            self.prev_z = self.z
            self.prev_theta = self.theta
            
            self.sensor_readings_ready = True
        except Exception as e:
            print(f"ERROR: Failed to get position from world file for {self.name}: {str(e)}")
            return False
        
        return True
    
    def get_odometry(self):
        """Get the current odometry data for the robot"""
        # Calculate velocities
        dt = self.robot.getTime() - self.last_pose_time
        if dt > 0:
            vx_world = (self.x - self.prev_x) / dt
            vy_world = (self.y - self.prev_y) / dt
            vz_world = (self.z - self.prev_z) / dt
            omega = (self.theta - self.prev_theta) / dt
        else:
            vx_world = 0.0
            vy_world = 0.0
            vz_world = 0.0
            omega = 0.0
        
        # Convert world velocities to robot frame
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)
        vx_robot = vx_world * cos_theta + vy_world * sin_theta
        vy_robot = -vx_world * sin_theta + vy_world * cos_theta
        
        return {
            'x': self.x,
            'y': self.y,
            'z': self.z,
            'theta': self.theta,
            'vx': vx_robot,
            'vy': vy_robot,
            'vz': vz_world,
            'omega': omega
        }
    
    def stop(self):
        """Stop the robot by setting all wheel speeds to 0"""
        for motor_name, motor in self.motors.items():
            if motor:
                motor.setVelocity(0.0)
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
    
    def check_timeout(self):
        """Check if command timeout has occurred and stop if necessary"""
        current_time = self.robot.getTime()
        if current_time - self.last_cmd_time > self.cmd_timeout:
            self.stop()
            return True
        return False
    
    def step(self):
        """Perform one control step"""
        self.check_timeout()
        self.update_position()
        return self.robot.step(self.timestep)

    def run(self):
        """Main control loop for the robot"""
        print(f"Starting AMR controller for robot {self.name}")
        
        if not hasattr(self, 'node'):
            print(f"ERROR: ROS2 node not initialized for {self.name}")
            return
        
        try:
            timestep = self.timestep
            
            while self.robot.step(timestep) != -1 and self.running:
                rclpy.spin_once(self.node, timeout_sec=0)
                self.update_position()
                
                current_time = self.robot.getTime()
                if current_time - self.last_cmd_time > self.cmd_timeout:
                    self.stop()
                
            print(f"AMR controller for {self.name} exiting")
            self.stop()
            
        except Exception as e:
            print(f"ERROR: Exception in AMR controller for {self.name}: {str(e)}")
        finally:
            self.stop()
            
            if hasattr(self, 'node') and self.node:
                self.node.destroy_node()
            
            print(f"AMR controller for {self.name} terminated")
    
    def cmd_vel_callback(self, msg):
        """ROS callback for velocity commands"""
        self.last_cmd_time = self.robot.getTime()
        
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z
        
        self.set_velocity(vx, vy, omega)
        
    def publish_odometry(self):
        """Publish odometry information to ROS"""
        if not hasattr(self, 'node') or not self.node:
            return
            
        try:
            now = self.node.get_clock().now()
            
            odom_msg = Odometry()
            odom_msg.header.stamp = now.to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = f'{self.name}/base_link'
            
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = self.z
            
            cy = math.cos(self.theta * 0.5)
            sy = math.sin(self.theta * 0.5)
            odom_msg.pose.pose.orientation.x = 0.0
            odom_msg.pose.pose.orientation.y = 0.0
            odom_msg.pose.pose.orientation.z = sy
            odom_msg.pose.pose.orientation.w = cy
            
            odom_msg.twist.twist.linear.x = self.vx
            odom_msg.twist.twist.linear.y = self.vy
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.z = self.omega
            
            self.odom_pub.publish(odom_msg)
            
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now.to_msg()
            tf_msg.header.frame_id = 'odom'
            tf_msg.child_frame_id = f'{self.name}/base_link'
            
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = self.z
            tf_msg.transform.rotation.x = 0.0
            tf_msg.transform.rotation.y = 0.0
            tf_msg.transform.rotation.z = sy
            tf_msg.transform.rotation.w = cy
            
            self.tf_broadcaster.sendTransform(tf_msg)
        except Exception as e:
            print(f"ERROR: Failed to publish odometry for {self.name}: {str(e)}")

class ROS2Interface(Node):
    """ROS2 interface for AMR controllers"""
    
    def __init__(self, controllers):
        """Initialize with a dictionary of AMR controllers"""
        robot_name = next(iter(controllers.keys()))
        super().__init__(f'amr_webots_interface_{robot_name}')
        self.controllers = controllers
        self.subscribers = {}
        self.odom_publishers = {}
        self.tf_broadcasters = {}
        
        for name, controller in self.controllers.items():
            self.get_logger().info(f"Creating subscriber for robot: {name}")
            self.subscribers[name] = self.create_subscription(
                Twist,
                f'/{name}/cmd_vel',
                lambda msg, robot_name=name: self.cmd_vel_callback(msg, robot_name),
                10
            )
            
            self.odom_publishers[name] = self.create_publisher(
                Odometry,
                f'/{name}/odom',
                10
            )
            
            self.tf_broadcasters[name] = TransformBroadcaster(self)
        
        self.odom_timer = self.create_timer(0.05, self.publish_odometry)
    
    def cmd_vel_callback(self, msg, robot_name):
        """Process velocity commands from ROS2"""
        if robot_name in self.controllers:
            vx = msg.linear.x
            vy = msg.linear.y
            omega = msg.angular.z
            
            self.get_logger().debug(f"Received cmd_vel for {robot_name}: linear=({vx}, {vy}), angular={omega}")
            
            self.controllers[robot_name].set_velocity(vx, vy, omega)
        else:
            self.get_logger().warn(f"Received command for unknown robot: {robot_name}")
    
    def publish_odometry(self):
        """Publish odometry and TF data for all robots"""
        for name, controller in self.controllers.items():
            try:
                odom_data = controller.get_odometry()
                
                odom_msg = Odometry()
                
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = f"{name}/base_link"
                
                odom_msg.pose.pose.position.x = odom_data['x']
                odom_msg.pose.pose.position.y = odom_data['y']
                odom_msg.pose.pose.position.z = odom_data['z']
                
                theta = odom_data['theta']
                odom_msg.pose.pose.orientation.x = 0.0
                odom_msg.pose.pose.orientation.y = 0.0
                odom_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
                odom_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
                
                odom_msg.twist.twist.linear.x = odom_data['vx']
                odom_msg.twist.twist.linear.y = odom_data['vy']
                odom_msg.twist.twist.linear.z = odom_data['vz']
                odom_msg.twist.twist.angular.z = odom_data['omega']
                
                self.odom_publishers[name].publish(odom_msg)
                
                tf_msg = TransformStamped()
                tf_msg.header.stamp = odom_msg.header.stamp
                tf_msg.header.frame_id = "odom"
                tf_msg.child_frame_id = f"{name}/base_link"
                
                tf_msg.transform.translation.x = odom_data['x']
                tf_msg.transform.translation.y = odom_data['y']
                tf_msg.transform.translation.z = odom_data['z']
                
                tf_msg.transform.rotation.x = 0.0
                tf_msg.transform.rotation.y = 0.0
                tf_msg.transform.rotation.z = math.sin(theta / 2.0)
                tf_msg.transform.rotation.w = math.cos(theta / 2.0)
                
                self.tf_broadcasters[name].sendTransform(tf_msg)
                
            except Exception as e:
                self.get_logger().error(f"Error publishing odometry for {name}: {str(e)}")

def main():
    """Main function to initialize and run the controller"""
    robot = Supervisor()
    robot_name = os.environ.get('WEBOTS_ROBOT_NAME', 'AMR')
    
    controller = AMRController(robot, name=robot_name)
    controller.run()

if __name__ == '__main__':
    main() 