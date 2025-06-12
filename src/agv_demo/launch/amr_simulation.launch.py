#!/usr/bin/env python3

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import EmitEvent, SetEnvironmentVariable
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generate launch description for AMR simulation.
    Launches Webots with AMR robots using the internal controllers.
    """
    # Get package directory
    try:
        package_dir = get_package_share_directory('agv_demo')
    except Exception as e:
        print(f"Error: Could not find agv_demo package: {str(e)}")
        sys.exit(1)
    
    # Check if required files exist
    world_file = os.path.join(package_dir, 'worlds', 'amr_world.wbt')
    
    if not os.path.exists(world_file):
        print(f"Error: World file not found: {world_file}")
        sys.exit(1)
    
    # Check controller files
    amr_controller = os.path.join(package_dir, 'agv_demo', 'amr_controller.py')
    ned_controller = os.path.join(package_dir, 'agv_demo', 'ned_controller.py')
    
    controllers_found = True
    
    if not os.path.exists(amr_controller):
        print(f"Warning: AMR controller file not found: {amr_controller}")
        controllers_found = False
    
    if not os.path.exists(ned_controller):
        print(f"Warning: Ned controller file not found: {ned_controller}")
        controllers_found = False
    
    if not controllers_found:
        print("Warning: Some controllers not found, but simulation will use internal controllers specified in world file")
    
    # Parameter for simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare the world argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to the Webots world file'
    )
    
    # Launch Webots - the controllers are specified in the world file
    webots = WebotsLauncher(
        world=world_file,
        mode='realtime'
    )
    
    # Event handlers for managing simulation lifecycle
    webots_started = RegisterEventHandler(
        OnProcessStart(
            target_action=webots,
            on_start=[LogInfo(msg='Webots started successfully with controllers')]
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
    
    # Set environment variables for the Ned Webots controller
    set_webots_robot_name = SetEnvironmentVariable(
        name='WEBOTS_ROBOT_NAME',
        value='Ned'
    )
    
    set_webots_controller_url = SetEnvironmentVariable(
        name='WEBOTS_CONTROLLER_URL',
        value='tcp://localhost:1234'
    )
    
    # Launch the Ned controller which will connect to Webots
    ned_webots_controller = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(package_dir, 'controllers', 'ned', 'ned_controller.py')
        ],
        name='ned_webots_controller',
        output='screen'
    )
    
    # Launch the Ned ROS2 interface node
    ned_ros2_controller = Node(
        package='agv_demo',
        executable='ned_controller',
        name='ned_ros2_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Combine all the launch entities
    launch_entities = [
        LogInfo(msg="Starting simulation with AMRs and Ned robot"),
        world_arg,
        set_webots_robot_name,
        set_webots_controller_url,
        webots,
        webots_started,
        shutdown_handler,
        ned_webots_controller,  # Launch the Webots controller first
        ned_ros2_controller     # Then launch the ROS2 interface
    ]
    
    return LaunchDescription(launch_entities)