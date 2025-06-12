#!/usr/bin/env python3

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import EmitEvent
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher

def generate_launch_description():
    """
    Generate launch description for AMR simulation.
    Launches Webots with AMR robots using the internal amr_controller.
    """
    # Get package directory
    try:
        package_dir = get_package_share_directory('amr_webots_sim')
    except Exception as e:
        print(f"Error: Could not find amr_webots_sim package: {str(e)}")
        sys.exit(1)
    
    # Check if required files exist
    world_file = os.path.join(package_dir, 'worlds', 'amr_world.wbt')
    
    if not os.path.exists(world_file):
        print(f"Error: World file not found: {world_file}")
        sys.exit(1)
    
    # Declare the world argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to the Webots world file'
    )
    
    # Launch Webots - the controller is specified in the world file
    webots = WebotsLauncher(
        world=world_file,
        mode='realtime'
    )
    
    # Event handlers for managing simulation lifecycle
    webots_started = RegisterEventHandler(
        OnProcessStart(
            target_action=webots,
            on_start=[LogInfo(msg='Webots started successfully with amr_controller')]
        )
    )
    
    # Create shutdown handler for clean exit
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=webots,
            on_exit=[
                LogInfo(msg='Webots exited, shutting down launch'),
                EmitEvent(event=Shutdown())
            ]
        )
    )
    
    # Combine all the launch entities
    launch_entities = [
        LogInfo(msg="Starting AMR simulation with internal controllers"),
        world_arg,
        webots,
        webots_started,
        shutdown_handler,
    ]
    
    return LaunchDescription(launch_entities)