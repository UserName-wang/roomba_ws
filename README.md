# Roomba ROS 2 Workspace

This workspace contains a collection of ROS 2 packages for controlling the iRobot Roomba/Create 2 robot. The packages provide comprehensive support for both real robot operation and simulation, with features including teleoperation, visualization, and hardware integration.

## Packages Overview

### 1. roomba_bringup

The main package for launching and configuring the Roomba robot with ROS 2. It provides launch files and configurations for various operational modes.

Key features:
- Launch files for real robot operation and simulation
- Controller configurations for differential drive control
- Robot description files (URDF)
- RViz configurations for visualization
- Teleoperation support with joystick
- TF transformation publishing
- Configurable joystick teleoperation

Main launch files:
- `roomba.launch.py` - Main launch file for real robot operation
- `simulation.launch.py` - Launch file for Gazebo simulation
- `teleop_control.launch.py` - Teleoperation with joystick
- `ros2_control.launch.py` - Core ros2_control framework integration
- `display.launch.py` - Robot model visualization in RViz

### 2. roomba_hardware_interface

A complete C++ driver implementation for the iRobot Roomba/Create 2 robot integrated as a ROS 2 hardware interface. This package enables full control of the robot through the serial interface using the Create 2 Open Interface (OI) protocol.

Key features:
- Full implementation of the Create 2 Open Interface protocol
- ROS 2 hardware interface integration for seamless robot control
- Complete sensor data access (bump sensors, cliff sensors, wheel drop sensors, etc.)
- Drive control with multiple modes (direct drive, radius drive, PWM drive)
- Motor control (main brush, side brush, vacuum)
- LED control
- Sound generation (songs and notes)
- Charging state monitoring and fault detection/clearing
- Smooth velocity control to prevent acceleration shocks

### 3. roomba_description

Contains robot description files (URDF) and visualization configurations for the Roomba robot. This package provides the necessary files for robot modeling and visualization in RViz and Gazebo.

Key features:
- Robot description files in URDF format
- Parameterized Xacro files for flexible configuration
- Visualization configurations for RViz
- Gazebo simulation support

## System Architecture

The Roomba ROS 2 system follows a modular architecture:

```
        User Input           Visualization
             |                     |
    [Joystick/Keyboard]      [RViz/Gazebo]
             |                     |
      [Teleoperation]        [Robot Model]
             |                     |
      [Velocity Commands]   [TF Transforms]
             |                     |
        [Controller]         [State Publisher]
             |                     |
    [Hardware Interface]   [Sensor Processing]
             |                     |
         [Roomba Robot]      [Sensor Data]
```

## Getting Started

### Prerequisites

- ROS 2 Humble
- Ubuntu 22.04
- Physical Roomba/Create 2 robot (for real robot operation)
- USB to serial cable (for real robot operation)
- Joystick (optional, for teleoperation)

### Installation

1. Install system dependencies:
   ```bash
   sudo apt install libserialport-dev
   ```

2. Install ROS 2 dependencies:
   ```bash
   cd ~/roomba_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace:
   ```bash
   cd ~/roomba_ws
   colcon build
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

### Usage

#### Real Robot Operation

1. Connect the Roomba to your computer via USB serial cable
2. Launch the main Roomba system:
   ```bash
   ros2 launch roomba_bringup roomba.launch.py
   ```

#### Simulation

To run the Roomba in Gazebo simulation:
```bash
ros2 launch roomba_bringup simulation.launch.py
```

#### Teleoperation

To teleoperate the Roomba with a joystick:
```bash
ros2 launch roomba_bringup teleop_control.launch.py
```

## Configuration

### Hardware Parameters

The hardware interface can be configured through parameters in the URDF:

```xml
<ros2_control name="RoombaSystem" type="system">
  <hardware>
    <plugin>roomba_hardware_interface/RoombaHardwareInterface</plugin>
    <param name="port">/dev/ttyUSB0</param>
    <param name="baud_rate">115200</param>
    <param name="max_acceleration">0.8</param>
    <param name="wheel_radius">0.03</param>
  </hardware>
</ros2_control>
```

### Controller Parameters

The differential drive controller is configured in `roomba_bringup/config/roomba_controllers.yaml` with parameters including:
- Wheel separation: 0.235m
- Wheel radius: 0.03m
- Velocity and acceleration limits
- Odometry publishing parameters

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## Package Structure Changes

The URDF files have been moved from the `roomba_bringup` package to the dedicated `roomba_description` package to maintain a clear separation of concerns. This change ensures that robot description files are centralized in one location and can be easily reused by other packages.

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file in each package for details.