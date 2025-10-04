# Roomba Driver Test Package

This package provides a test node for the Roomba hardware driver and demonstrates basic control of a Roomba robot using ROS 2.

## Overview

The `roomba_driver_test` package contains:
- A C++ node that interfaces with the Roomba hardware via serial communication
- A launch file that starts the driver test node and optional joystick teleoperation
- Example implementation of sending velocity commands to the Roomba robot

## Features

- Direct hardware communication with Roomba via serial port
- Control Roomba using `geometry_msgs/Twist` messages on the `cmd_vel` topic
- Joystick teleoperation support using `joy` and `teleop_twist_joy` packages
- Battery and charging status monitoring
- Safe mode operation for protected Roomba control

## Prerequisites

- ROS 2 (Foxy or later recommended)
- Roomba robot with serial interface
- `joy` and `teleop_twist_joy` packages for joystick control
- [roomba_hardware_interface](file:///roomba/roomba_ws/src/roomba_hardware_interface/package.xml) package

## Usage

### Basic launch

```bash
ros2 launch roomba_driver_test test_driver.launch.py
```

### Launch with custom serial port

```bash
ros2 launch roomba_driver_test test_driver.launch.py port:=/dev/ttyUSB1
```

### Launch without joystick

```bash
ros2 launch roomba_driver_test test_driver.launch.py use_joystick:=false
```

### Manual control

To manually control the Roomba, publish to the `cmd_vel` topic:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.1}}"
```

## Parameters

- `port` (string, default: "/dev/ttyUSB0") - Serial port for Roomba connection
- `baud_rate` (int, default: 115200) - Baud rate for serial communication
- `use_joystick` (bool, default: true) - Enable/disable joystick teleoperation

## Nodes

### roomba_driver_test

Main test node that connects to Roomba hardware and processes velocity commands.

**Subscriptions:**
- `cmd_vel` - Receives velocity commands (geometry_msgs/Twist)

**Publishers:**
- `roomba_status` - Publishes battery and charging status (std_msgs/String)

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.