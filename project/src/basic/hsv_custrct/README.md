# Hsvcustrct

## Description
HSVCU structured drawing objects library for ROS2 applications.

## Features
- Provides structured drawing objects for computer vision applications
- Implements various geometric shapes and drawing utilities
- Integrates with OpenCV for image processing

## Dependencies
- ROS2 Humble or later
- rclcpp
- std_msgs
- OpenCV

## Building
To build this package:

```bash
colcon build --packages-select Hsvcustrct
```

## Usage
After building, source the workspace and use the library in your ROS2 nodes.

## API Documentation
The library provides classes for:
- Geometric shapes (Circle, Rectangle, Polygon, etc.)
- Drawing operations
- Image processing utilities

## Directory Structure
- `include/` - Header files
- `src/` - Source files
- `test/` - Test files