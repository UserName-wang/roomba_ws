# Roomba Bringup Package

This package provides launch files and configurations to easily bring up and control the iRobot Roomba/Create 2 robot using ROS 2. It includes configurations for both real robot operation and simulation.

## Overview

The Roomba Bringup package enables quick deployment of the Roomba robot with ROS 2, providing:

- Launch files for various operational modes
- Controller configurations for differential drive control
- Robot description files (URDF) - now in the `roomba_description` package
- RViz configurations for visualization
- Teleoperation support with joystick

## Features

- Real robot control using the `roomba_hardware_interface` package
- Gazebo simulation support
- Differential drive controller configuration
- Joint state publisher and robot state publisher
- RViz visualization configurations
- Teleoperation with joystick
- TF transformation publishing for robot visualization
- Smooth velocity control to prevent acceleration shocks
- Configurable joystick teleoperation

## File Structure

```
roomba_bringup/
├── config/
│   ├── roomba_controllers.yaml  # Controller configurations
│   └── roomba.rviz              # RViz configuration
├── launch/
│   ├── display.launch.py        # Display robot model
│   ├── gazebo.launch.py         # Launch in Gazebo simulation
│   ├── roomba.launch.py         # Main launch file for real robot
│   ├── ros2_control.launch.py   # Launch with ros2_control
│   ├── simulation.launch.py     # Simulation launch file
│   ├── teleop_control.launch.py # Teleoperation with joystick
│   └── tf_demo.launch.py        # TF demonstration
├── roomba_bringup/
│   ├── robot_simulator.py       # Robot simulator node
│   ├── roomba_controller.py     # Roomba controller node
│   ├── roomba_node.py           # Main Roomba node
│   ├── tf_publisher.py          # TF publisher
│   └── test_controller.py       # Test controller
├── package.xml                  # Package metadata
├── CMakeLists.txt               # Build configuration
└── README.md                    # This file
```

## Dependencies

- ROS 2 Humble
- `roomba_hardware_interface` package
- `roomba_description` package (for robot description files)
- `controller_manager`
- `diff_drive_controller`
- `joint_state_broadcaster`
- `robot_state_publisher`
- `rviz2`
- `gazebo_ros_pkgs` (for simulation)
- `joy` and `teleop_twist_joy` (for teleoperation)

## Installation

1. Clone the repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url> roomba_bringup
   ```

2. Install dependencies:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select roomba_bringup
   ```

4. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Real Robot Operation

To operate with a real Roomba robot:

1. Connect the Roomba to your computer via USB serial cable
2. Launch the main Roomba system:
   ```bash
   ros2 launch roomba_bringup roomba.launch.py
   ```

This will start:
- The hardware interface to communicate with the Roomba
- The controller manager with differential drive controller
- The robot state publisher (using URDF from `roomba_description` package)
- RViz for visualization

### Simulation

To run the Roomba in Gazebo simulation:
```bash
ros2 launch roomba_bringup simulation.launch.py
```

This will start:
- Gazebo with the Roomba model (using URDF from `roomba_description` package)
- The controller manager with differential drive controller
- The robot state publisher
- RViz for visualization

### Teleoperation

To teleoperate the Roomba with a joystick:
```bash
ros2 launch roomba_bringup teleop_control.launch.py
```

This will start:
- The Roomba system (real or simulated)
- The joy node to read joystick inputs
- The teleop_twist_joy node to convert joystick inputs to velocity commands

### Launch with Joystick Support

You can also enable joystick teleoperation with the main launch files using parameters:

```bash
# Launch with joystick support
ros2 launch roomba_bringup roomba.launch.py use_joystick:=true

# Launch simulation with joystick support
ros2 launch roomba_bringup simulation.launch.py use_joystick:=true
```

### Individual Launch Files

The package also provides individual launch files for specific purposes:

- `display.launch.py`: Display the robot model in RViz
- `gazebo.launch.py`: Launch Gazebo with the Roomba model
- `ros2_control.launch.py`: Launch the ros2_control framework with the Roomba
- `tf_demo.launch.py`: Demonstrate TF transformations

## Smooth Velocity Control

The Roomba hardware interface implements smooth velocity control to prevent acceleration shocks that could damage the robot or cause discomfort. This feature is especially important when using teleoperation, as sudden joystick movements could result in harsh robot movements.

The smooth velocity control works by limiting the rate at which velocity commands can change. When a new velocity command is received, the hardware interface calculates the maximum allowed velocity change based on the configured maximum acceleration and the time elapsed since the last update:

```
max_velocity_change = max_acceleration * time_step
```

If the difference between the target velocity and current velocity exceeds this limit, the velocity is adjusted incrementally until it reaches the target value. This results in smooth acceleration and deceleration, protecting the robot's mechanical components and providing a better user experience.

### Configuration

The smooth velocity control can be configured through parameters in the URDF:

```xml
<ros2_control name="RoombaSystem" type="system">
  <hardware>
    <plugin>roomba_hardware_interface/RoombaHardwareInterface</plugin>
    <param name="port">/dev/ttyUSB0</param>
    <param name="baud_rate">115200</param>
    <param name="max_acceleration">0.8</param>
    <param name="wheel_radius">0.03</param>
  </hardware>
  <!-- ... -->
</ros2_control>
```

- `max_acceleration`: Maximum acceleration for smooth velocity control in rad/s² (default: 0.5)
- `wheel_radius`: Wheel radius in meters (default: 0.03)

Additionally, the diff_drive_controller is configured with matching acceleration limits in `config/roomba_controllers.yaml`:

```yaml
diff_drive_controller:
  ros__parameters:
    linear:
      x:
        has_acceleration_limits: true
        max_acceleration: 0.8
        min_acceleration: -0.8
    angular:
      z:
        has_acceleration_limits: true
        max_acceleration: 0.6
        min_acceleration: -0.6
```

These settings ensure consistent behavior between the controller and the hardware interface.

## Controller Configuration

The differential drive controller is configured in `config/roomba_controllers.yaml` with the following parameters:
- Wheel separation: 0.235m
- Wheel radius: 0.03m
- Velocity limits for linear and angular movement
- Acceleration limits
- Odometry publishing parameters

## URDF Description

The robot description files (URDF) have been moved to the `roomba_description` package to maintain a clear separation of concerns. The launch files in this package now reference the URDF files from the `roomba_description` package.

## RViz Configuration

The RViz configuration in `config/roomba.rviz` includes:
- RobotModel display
- TF display
- Grid display
- Configured viewpoints

## Nodes

### roomba_controller

Converts velocity commands (`cmd_vel`) to wheel velocity commands for the differential drive controller.

### robot_simulator

Simulates robot movement in the absence of real hardware by integrating wheel velocities to produce TF transformations.

### tf_publisher

Publishes TF transformations for robot visualization (used in demonstrations).

### test_controller

A simple test node for controller functionality.

## Topics

- `/cmd_vel`: Input velocity commands (geometry_msgs/Twist)
- `/diff_drive_controller/cmd_vel`: Velocity commands for the differential drive controller
- `/diff_drive_controller/odom`: Odometry output from the controller (nav_msgs/Odometry)
- `/joint_states`: Joint states (sensor_msgs/JointState)

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.