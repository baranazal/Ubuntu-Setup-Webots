#!/usr/bin/env python3

"""
ROS2-integrated controller for the conveyor belt using supervisor API
Allows controlling the speed and toggling the belt via ROS2 topics
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from controller import Supervisor
import sys
import os

class BeltSupervisor(Node):
    """
    Supervisor controller that interfaces with the conveyor belt
    and exposes ROS2 topics for controlling it
    """
    def __init__(self):
        # Initialize the supervisor
        try:
            self.supervisor = Supervisor()
            
            # Initialize as a ROS2 node
            super().__init__('belt_supervisor')
            
            # Get the timestep
            self.timestep = int(self.supervisor.getBasicTimeStep())
            self.get_logger().info(f'Belt supervisor started with timestep {self.timestep}ms')
            
            # Find the conveyor belt node in the scene
            self.belt_node = self.supervisor.getFromDef("conveyor_belt")
            if not self.belt_node:
                self.get_logger().error('Could not find conveyor_belt node. Make sure it has a DEF name in the world file.')
                self.get_logger().info('Trying to find it by name instead...')
                # Try to find by name
                root = self.supervisor.getRoot()
                if root:
                    children_field = root.getField("children")
                    if children_field:
                        for i in range(children_field.getCount()):
                            node = children_field.getMFNode(i)
                            if node and node.getTypeName() == "ConveyorBelt":
                                self.belt_node = node
                                self.get_logger().info('Found conveyor belt by type!')
                                break
                            elif node and node.getField("name") and node.getField("name").getSFString() == "conveyor belt":
                                self.belt_node = node
                                self.get_logger().info('Found conveyor belt by name!')
                                break
                
            if not self.belt_node:
                self.get_logger().error('Could not find conveyor belt node by any method. Cannot continue.')
                return
            
            # Print information about the belt node
            self.get_logger().info(f"Belt node found of type: {self.belt_node.getTypeName()}")
            
            # Try to directly access the speed field
            self.speed_field = self.belt_node.getField("speed")
            if not self.speed_field:
                self.get_logger().error('Could not find speed field in conveyor belt. Trying alternative methods...')
                
                # Try to get controllerArgs which might contain the speed
                controller_args_field = self.belt_node.getField("controllerArgs")
                if controller_args_field:
                    self.get_logger().info("Found controllerArgs field, will try to modify it")
                    # We'll need to use a different strategy to control the belt
                    # For now, we'll just track state locally
                    self.speed_field = None
                else:
                    self.get_logger().error("Could not find any way to control the belt speed. Cannot continue.")
                    return
            else:
                self.get_logger().info("Successfully found speed field in conveyor belt")
            
            # Initial belt state - either from field or default
            if self.speed_field:
                # Initialize with 0 speed (stopped) rather than reading existing value
                self.speed_field.setSFFloat(0.0)
                self.current_speed = 0.0
            else:
                self.current_speed = 0.0  # Default stopped
            
            self.is_running = False  # Default to stopped state
            self.get_logger().info(f'Belt speed initialized to: {self.current_speed} (stopped)')
            
            # Store the default speed value to use when toggling
            self.default_speed = 0.5  # Default speed when started
            
            # Create ROS2 subscribers for controlling the belt
            self.speed_subscriber = self.create_subscription(
                Float64, '/conveyor_belt/set_speed', self.speed_callback, 10)
                
            self.toggle_subscriber = self.create_subscription(
                Bool, '/conveyor_belt/toggle', self.toggle_callback, 10)
                
            # Create publishers to report current state
            self.speed_publisher = self.create_publisher(
                Float64, '/conveyor_belt/current_speed', 10)
                
            self.state_publisher = self.create_publisher(
                Bool, '/conveyor_belt/is_running', 10)
                
            # Timer for main control loop and publishing state
            self.timer = self.create_timer(0.5, self.publish_state)
            
            self.get_logger().info('Belt supervisor initialized and ready for commands')
            
        except Exception as e:
            if hasattr(self, 'get_logger'):
                self.get_logger().error(f'Error initializing belt supervisor: {str(e)}')
            else:
                print(f'Error initializing belt supervisor: {str(e)}')
            
    def speed_callback(self, msg):
        """Handle speed change requests"""
        try:
            new_speed = float(msg.data)
            self.get_logger().info(f'Setting conveyor belt speed to {new_speed}')
            
            if self.speed_field:
                # Set the speed field directly
                self.speed_field.setSFFloat(new_speed)
                self.get_logger().info(f'Belt speed set to {new_speed} via speed field')
            else:
                # We don't have direct access to speed field
                # Just update our internal state - in reality the belt won't change speed
                self.get_logger().info(f'No direct access to belt speed field. Logging change only.')
                
            # Update internally tracked state
            self.current_speed = new_speed
            self.is_running = new_speed > 0
            
            # If speed is set to a positive value, update the default speed
            if new_speed > 0:
                self.default_speed = new_speed
                
        except Exception as e:
            self.get_logger().error(f'Error setting conveyor belt speed: {str(e)}')
    
    def toggle_callback(self, msg):
        """Handle toggle requests (turn belt on/off)"""
        try:
            should_run = msg.data
            self.get_logger().info(f'{"Starting" if should_run else "Stopping"} conveyor belt')
            
            if should_run and not self.is_running:
                # Turn on the belt using the default speed
                if self.speed_field:
                    self.speed_field.setSFFloat(self.default_speed)
                    self.get_logger().info(f'Belt started at speed {self.default_speed}')
                else:
                    self.get_logger().info('No direct access to belt speed. Logging state change only.')
                self.current_speed = self.default_speed
                self.is_running = True
            elif not should_run and self.is_running:
                # Turn off the belt
                if self.speed_field:
                    self.speed_field.setSFFloat(0.0)
                    self.get_logger().info('Belt stopped')
                else:
                    self.get_logger().info('No direct access to belt speed. Logging state change only.')
                self.current_speed = 0.0
                self.is_running = False
                
        except Exception as e:
            self.get_logger().error(f'Error toggling conveyor belt: {str(e)}')
    
    def publish_state(self):
        """Publish current belt state to ROS topics"""
        try:
            # Publish current speed
            speed_msg = Float64()
            speed_msg.data = self.current_speed
            self.speed_publisher.publish(speed_msg)
            
            # Publish current running state
            state_msg = Bool()
            state_msg.data = self.is_running
            self.state_publisher.publish(state_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing belt state: {str(e)}')
    
    def run(self):
        """Main control loop"""
        try:
            # Main loop
            while self.supervisor.step(self.timestep) != -1:
                rclpy.spin_once(self, timeout_sec=0)
                
        except Exception as e:
            self.get_logger().error(f'Error in belt supervisor main loop: {str(e)}')
        finally:
            if hasattr(self, 'destroy_node'):
                self.destroy_node()
            
            self.get_logger().info('Belt supervisor shutting down')


def main():
    rclpy.init(args=sys.argv)
    controller = BeltSupervisor()
    controller.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
