# Roomba Description Package

This package contains the URDF and configuration files for the Roomba robot.

## Package Structure

- `urdf/` - Contains the URDF and xacro files describing the robot
- `config/` - Contains RViz configuration files
- `launch/` - Contains launch files for displaying the robot model

## Usage

To display the robot model in RViz:

```bash
# Using standard URDF
ros2 launch roomba_description display.launch.py

# Using xacro
ros2 launch roomba_description display_xacro.launch.py
```

## Files

- `roomba.urdf` - Standard URDF file
- `roomba.urdf.xacro` - Parameterized xacro version
- `roomba.rviz` - RViz configuration for visualizing the robot