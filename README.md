# AMR (Autonomous Mobile Robot) ROS2 Webots Simulation

This project implements an Autonomous Mobile Robot (AMR) simulation using ROS2 and Webots. The AMR features four-wheel mecanum drive configuration for enhanced maneuverability and sophisticated navigation capabilities.

## Features

### Robot Configuration
- Four-wheel mecanum drive system
- Enhanced chassis design with industrial-grade components
- Sensor suite including:
  - 360° LIDAR (5m range)
  - Front-facing camera
  - GPS for position tracking
  - IMU for orientation sensing

### Navigation System
- Advanced PID-controlled movement
- Separate rotation and movement phases for precise navigation
- Obstacle avoidance with dynamic path planning
- Synchronized wheel control for smooth motion
- Automatic speed adjustment based on obstacles

### Movement Capabilities
- Forward/backward motion
- Lateral movement
- In-place rotation
- Combined movements with automatic speed correction
- Smooth acceleration and deceleration

## Installation Guide

### Prerequisites
- Ubuntu 22.04 or later
- ROS2 Humble
- Webots R2023b or later
- Python 3.8+

### ROS2 Dependencies
```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-webots-ros2 \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox
```

### Project Setup
1. Create a ROS2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone the repository:
```bash
git clone ...
```

3. Build the workspace:
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

4. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

### Running the Simulation

1. Launch the AMR simulation:
```bash
ros2 launch amr_webots_sim amr_simulation.launch.py
```

2. Launch RViz2 for visualization (optional):
```bash
ros2 launch amr_webots_sim amr_rviz.launch.py
```

## Configuration

### Robot Parameters
- Wheel radius: 0.055m
- Wheel base (x): 0.34m
- Wheel base (y): 0.30m
- Max wheel speed: 15.0 rad/s
- Max torque: 1.0 Nm

### Navigation Parameters
- Safe distance: 1.5m
- Critical distance: 0.5m
- Position threshold: 0.1m
- Rotation threshold: 5 degrees

### Speed Control
- Max linear speed: 1.0 m/s
- Min linear speed: 0.1 m/s
- Max rotation speed: 1.5 rad/s
- Min rotation speed: 0.2 rad/s

### PID Controllers
- Rotation control:
  - Kp: 1.0
  - Ki: 0.01
  - Kd: 0.1
- Position control:
  - Kp: 1.0
  - Ki: 0.01
  - Kd: 0.1

## Project Structure
```
amr_webots_sim/
├── launch/
│   ├── amr_simulation.launch.py
│   └── amr_rviz.launch.py
├── worlds/
│   └── amr_world.wbt
├── amr_webots_sim/
│   └── amr_controller.py
├── config/
│   └── amr_params.yaml
└── README.md
```

## Key Features Explained

### Wheel Synchronization
The AMR implements sophisticated wheel synchronization to ensure smooth movement:
- Direction consistency check for straight movement
- Proportional speed scaling for all wheels
- Acceleration limiting for smooth transitions
- Automatic speed correction during rotation

### Navigation System
The navigation system uses a state machine approach:
1. ROTATING: Aligns the robot with the target
2. MOVING: Moves toward the target with continuous small corrections
3. OBSTACLE_AVOIDING: Handles obstacle avoidance when needed

### Obstacle Avoidance
- 360° obstacle detection using LIDAR
- Dynamic speed adjustment based on obstacle proximity
- Smooth avoidance maneuvers using mecanum drive capabilities
- Automatic return to navigation when path is clear

## Troubleshooting

### Common Issues
1. Wheel movement issues:
   - Check wheel motor configurations in `amr_world.wbt`
   - Verify PID parameters in `amr_controller.py`

2. Navigation problems:
   - Adjust speed parameters for smoother movement
   - Fine-tune PID controllers for better precision

3. Simulation crashes:
   - Ensure all ROS2 dependencies are installed
   - Check for proper workspace setup

## Contributing
Feel free to contribute to this project by:
1. Forking the repository
2. Creating a feature branch
3. Submitting a pull request

## License
This project is licensed under the MIT License - see the LICENSE file for details. 