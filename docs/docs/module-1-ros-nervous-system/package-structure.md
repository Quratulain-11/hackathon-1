---
sidebar_position: 2
title: "ROS 2 Package Structure"
---

# ROS 2 Package Structure

## Understanding the Package Architecture

A well-structured ROS 2 package is crucial for maintainable and scalable robotics applications. In this section, we'll explore the recommended structure for our robot control packages and understand the purpose of each component.

## Standard ROS 2 Package Layout

```
robot_control/                 # Package name (lowercase, underscore)
├── CMakeLists.txt            # Build configuration for C++
├── package.xml               # Package metadata and dependencies
├── setup.py                  # Python setup configuration
├── setup.cfg                 # Installation configuration
├── resource/robot_control    # Resource files
├── test/                     # Test files
│   ├── __init__.py
│   └── test_copyright.py
├── robot_control/            # Python package directory
│   ├── __init__.py
│   ├── nodes/                # Individual ROS nodes
│   ├── utils/                # Utility functions
│   ├── msgs/                 # Custom message definitions
│   └── config/               # Configuration files
└── launch/                   # Launch files
    └── robot_control.launch.py
```

## Package Metadata (package.xml)

The `package.xml` file contains essential metadata about your package:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_control</name>
  <version>0.0.1</version>
  <description>ROS 2 package for humanoid robot control</description>
  <maintainer email="developer@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>message_runtime</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Build Configuration (setup.py)

The `setup.py` file configures how your Python package is built and installed:

```python
from setuptools import setup
from glob import glob
import os

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 package for humanoid robot control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_publisher = robot_control.sensor_publisher:main',
            'actuator_subscriber = robot_control.actuator_subscriber:main',
            'communication_example = robot_control.communication_example:main',
        ],
    },
)
```

## Node Organization

Organizing nodes in a logical structure improves maintainability:

### Communication Nodes
- `sensor_publisher.py` - Publishes sensor data from various robot sensors
- `actuator_subscriber.py` - Subscribes to actuator commands and executes them
- `transform_broadcaster.py` - Publishes coordinate transforms between robot frames

### Control Nodes
- `joint_controller.py` - Manages joint position, velocity, and effort control
- `navigation_controller.py` - Handles path planning and navigation commands
- `gripper_controller.py` - Controls end-effectors and grippers

### Diagnostic Nodes
- `health_monitor.py` - Monitors system health and reports issues
- `performance_tracker.py` - Tracks system performance metrics
- `safety_manager.py` - Implements safety checks and emergency stops

## Configuration Management

Configuration files allow for easy customization without code changes:

### YAML Configuration Example

```yaml
# config/robot_control_params.yaml
robot_control:
  ros__parameters:
    # Joint control parameters
    joint_update_rate: 50.0
    max_joint_velocity: 2.0
    joint_tolerance: 0.01

    # Communication parameters
    sensor_timeout: 0.1
    command_timeout: 0.5

    # Safety parameters
    max_velocity: 1.0
    max_acceleration: 2.0
    emergency_stop_threshold: 0.5
```

### Loading Parameters in Nodes

```python
# In your node's __init__ method
self.declare_parameter('joint_update_rate', 50.0)
self.declare_parameter('max_joint_velocity', 2.0)
self.declare_parameter('joint_tolerance', 0.01)

self.joint_update_rate = self.get_parameter('joint_update_rate').value
self.max_joint_velocity = self.get_parameter('max_joint_velocity').value
self.joint_tolerance = self.get_parameter('joint_tolerance').value
```

## Launch File Organization

Launch files coordinate multiple nodes and set up the complete system:

### Basic Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('robot_control')

    # Load parameters from YAML file
    params_file = os.path.join(pkg_share, 'config', 'robot_control_params.yaml')

    ld = LaunchDescription()

    # Sensor publisher node
    sensor_publisher = Node(
        package='robot_control',
        executable='sensor_publisher',
        name='sensor_publisher',
        parameters=[params_file],
        output='screen',
        respawn=True  # Restart if it crashes
    )

    # Actuator subscriber node
    actuator_subscriber = Node(
        package='robot_control',
        executable='actuator_subscriber',
        name='actuator_subscriber',
        parameters=[params_file],
        output='screen',
        respawn=True
    )

    # Add nodes to launch description
    ld.add_action(sensor_publisher)
    ld.add_action(actuator_subscriber)

    return ld
```

## Testing Structure

A comprehensive testing strategy ensures code quality:

### Unit Tests

```python
# test/test_robot_control.py
import unittest
import rclpy
from robot_control.joint_controller import JointController

class TestJointController(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = JointController()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_joint_command_validation(self):
        # Test that invalid joint commands are rejected
        result = self.node.validate_joint_command([1.0, 2.0, 3.0])
        self.assertTrue(result)

    def test_joint_limit_checking(self):
        # Test that joint limits are enforced
        result = self.node.check_joint_limits([100.0, 200.0, 300.0])
        self.assertFalse(result)

if __name__ == '__main__':
    unittest.main()
```

## Best Practices

1. **Consistent Naming**: Use clear, descriptive names for packages, nodes, and topics
2. **Modular Design**: Each node should have a single, well-defined responsibility
3. **Error Handling**: Implement robust error handling and recovery mechanisms
4. **Documentation**: Document all public interfaces and complex algorithms
5. **Testing**: Write unit tests for all critical functionality
6. **Logging**: Use appropriate log levels for debugging and monitoring
7. **Configuration**: Use parameters for configurable values rather than hardcoding

## Summary

The package structure provides the foundation for scalable and maintainable ROS 2 applications. By following these conventions, you ensure that your code integrates well with the broader ROS 2 ecosystem and remains maintainable as your robot system grows in complexity.

In the next section, we'll implement specific communication patterns and create practical examples of publisher-subscriber and service interactions.