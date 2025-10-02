# Roomba Hardware Interface for ROS 2

This package provides a complete C++ driver implementation for the iRobot Roomba/Create 2 robot, integrated as a ROS 2 hardware interface. It enables full control of the robot through the serial interface, following the official Create 2 Open Interface (OI) protocol specification.

## Features

- Full implementation of the Create 2 Open Interface protocol
- ROS 2 hardware interface integration for seamless robot control
- Complete sensor data access (bump sensors, cliff sensors, wheel drop sensors, etc.)
- Drive control with multiple modes (direct drive, radius drive, PWM drive)
- Motor control (main brush, side brush, vacuum)
- LED control
- Sound generation (songs and notes)
- Charging state monitoring and fault detection/clearing
- Differential drive control with velocity smoothing
- Example application demonstrating all major functionalities

## File Structure

```
roomba_hardware_interface/
├── include/
│   └── roomba_hardware_interface/
│       ├── roomba_hardware_driver.hpp  # Main driver header file
│       ├── roomba_hardware_interface.hpp  # ROS 2 hardware interface
│       └── visibility_control.h        # Symbol visibility control
├── src/
│   ├── roomba_hardware_driver.cpp      # Main driver implementation
│   ├── roomba_hardware_interface.cpp   # ROS 2 hardware interface implementation
│   └── roomba_example.cpp              # Example application
├── package.xml                         # Package metadata
├── CMakeLists.txt                      # Build configuration
├── roomba_hardware_interface.xml       # Plugin description
└── README.md                           # This file
```

## Dependencies

- ROS 2 (Humble or later recommended)
- libserialport-dev (for serial communication)
- Hardware supporting ROS 2 control system interfaces

To install the required system dependency:

```bash
sudo apt install libserialport-dev
```

## Installation

1. Clone this repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url> roomba_hardware_interface
   ```

2. Install dependencies:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select roomba_hardware_interface
   ```

4. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Hardware Interface

To use the Roomba hardware interface with ROS 2 control systems, you need to configure your URDF and controller YAML files appropriately. The hardware interface provides:

- Velocity command interfaces for differential drive
- Position state interfaces for wheel encoders

Example hardware parameters in URDF:
```xml
<ros2_control name="RoombaSystem" type="system">
  <hardware>
    <plugin>roomba_hardware_interface/RoombaHardwareInterface</plugin>
    <param name="port">/dev/ttyUSB0</param>
    <param name="baud_rate">115200</param>
  </hardware>
  <!-- Define joints and interfaces -->
</ros2_control>
```

### Direct Driver Usage

You can also use the driver directly in your C++ applications:

```cpp
#include "roomba_hardware_interface/roomba_hardware_driver.hpp"

auto roomba = std::make_unique<roomba_hardware_interface::RoombaHardware>("/dev/ttyUSB0", 115200);
roomba->connect();
roomba->start();
roomba->safe_mode();
// ... use other methods
roomba->disconnect();
```

### Running the Example

To run the example application that demonstrates all driver capabilities:

```bash
ros2 run roomba_hardware_interface roomba_example
```

Note: Make sure you have the proper permissions to access the serial port. You might need to add your user to the dialout group:
```bash
sudo adduser $USER dialout
```

Then log out and log back in for the changes to take effect.

## Smooth Velocity Control

The hardware interface implements smooth velocity control to prevent acceleration shocks that could damage the robot or cause discomfort. This is achieved by limiting the rate at which velocity commands can change.

When a new velocity command is received, the hardware interface calculates the maximum allowed velocity change based on the configured maximum acceleration and the time elapsed since the last update:

```
max_velocity_change = max_acceleration * time_step
```

If the difference between the target velocity and current velocity exceeds this limit, the velocity is adjusted incrementally until it reaches the target value. This results in smooth acceleration and deceleration, protecting the robot's mechanical components and providing a better user experience.

To configure the smooth velocity control, adjust the `max_acceleration` parameter in your URDF. Lower values result in smoother but slower acceleration, while higher values allow for more responsive but potentially harsher movements.

## API Overview

The driver implements all major Create 2 OI commands:

### Basic Commands
- `start()` - Start the Open Interface
- `reset()` - Reset the robot
- `stop()` - Stop the Open Interface
- `safe_mode()` - Set safe mode
- `full_mode()` - Set full mode

### Cleaning Commands
- `clean()` - Start normal cleaning
- `max_clean()` - Start maximum cleaning
- `spot_clean()` - Start spot cleaning
- `seek_dock()` - Seek docking station
- `power_down()` - Power down the robot

### Movement Commands
- `drive(velocity, radius)` - Drive with velocity and turning radius
- `drive_direct(right_velocity, left_velocity)` - Direct wheel velocity control
- `drive_pwm(right_pwm, left_pwm)` - Direct wheel PWM control

### Motor Control
- `motors(motor_bits)` - Control various motors with bit flags
- `pwm_motors(main_brush_pwm, side_brush_pwm, vacuum_pwm)` - PWM motor control

### LED Control
- `leds(led_bits, power_color, power_intensity)` - Control LEDs

### Sound
- `song(song_number, notes)` - Define a song
- `play(song_number)` - Play a song

### Sensor Data
- `read_charging_state()` - Get charging state
- `has_fault()` - Check for charging fault
- `clear_fault()` - Clear charging fault
- Various sensor reading methods for bumps, cliffs, wheels, etc.
- Distance and angle traveled
- Battery information

## Sensor Data Access

The driver provides comprehensive access to all Create 2 sensors:

- Bump and wheel drop sensors
- Cliff sensors (left, front left, front right, right)
- Wall sensor
- Virtual wall sensor
- Wheel overcurrent sensors
- Distance and angle traveled
- Battery status (voltage, current, temperature, charge, capacity)
- Charging state
- OI mode
- Song playing status

## Fault Handling

The driver implements fault detection and clearing capabilities:

- `has_fault()` - Checks if the robot is in a charging fault condition
- `clear_fault()` - Attempts to clear a charging fault by resetting the robot

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

## Acknowledgments

- Based on the iRobot Create 2 Open Interface Specification
- Uses libserialport for cross-platform serial communication