#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import controller_url_prefix

def generate_launch_description():
    package_dir = get_package_share_directory('conveyor_belt_ros')
    robot_description_path = os.path.join(package_dir, 'resource', 'conveyor_belt.yaml')
    world_path = os.path.join(package_dir, 'worlds', 'conveyor_world.wbt')
    
    webots = WebotsLauncher(
        world=world_path,
        mode='realtime',
    )
    
    # This is the correct way to use the Webots ROS 2 driver
    conveyor_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={
            'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'MainConveyor'
        },
        parameters=[
            {'robot_description': robot_description_path}
        ]
    )
    
    # Interface node
    interface_node = Node(
        package='conveyor_belt_ros',
        executable='conveyor_interface',
        name='conveyor_interface_node',
        parameters=[
            {'conveyor_id': 'MainConveyor'}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        webots,
        conveyor_driver,
        interface_node,
    ])