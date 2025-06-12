# AMR Webots Simulation

This package provides a simulation of Autonomous Mobile Robots (AMRs) in the Webots environment.

## Components

1. **Webots Simulation**: Simulates 5 AMR robots in a customizable environment.
2. **ROS2 Integration**: Uses ROS2 for robot control and communication.

## Prerequisites

- ROS2 (tested on Humble)
- Webots R2023b or later
- Python 3.8 or later

## Installation

1. Clone this repository into your ROS2 workspace
2. Build the workspace with colcon:
   ```
   colcon build --symlink-install
   ```
3. Source the workspace:
   ```
   source install/setup.bash
   ```

## Usage

### Running the Simulation

To run the Webots simulation:

```bash
ros2 launch amr_webots_sim amr_simulation.launch.py
```

This will start the Webots simulation with 5 AMR robots.

### Robot IDs

The simulation includes 5 AMR robots with the following IDs:

- **1**: AMR (blue)
- **2**: AMR2 (red)
- **3**: AMR3 (green)
- **4**: AMR4 (yellow)
- **5**: AMR5 (purple)

## Customization

### Modifying the World

The Webots world file is located in `worlds/amr_world.wbt`. You can edit this file in Webots to add or remove objects, change the environment, etc.

### Modifying Robot Behavior

The robot controller is defined in `controllers/amr_controller/amr_controller.py`. You can modify this file to change how the robots behave.

## License

Apache License 2.0