---
sidebar_position: 1
title: "ROS 2 Nervous System"
---

# Module 1: ROS 2 Nervous System

## Understanding the Robot's Communication Network

In this module, we'll build the communication backbone of our humanoid robot - its "nervous system." Just as the human nervous system enables different parts of the body to communicate and coordinate, ROS 2 (Robot Operating System 2) provides the infrastructure for different robot components to work together seamlessly.

## Learning Objectives

By the end of this module, you will:
- Understand the fundamental concepts of ROS 2 communication patterns
- Create ROS 2 packages for different robot subsystems
- Implement publishers and subscribers for sensor and actuator data
- Design services and actions for synchronous and asynchronous operations
- Build launch files to coordinate multiple nodes
- Debug communication issues in distributed systems

## The ROS 2 Architecture

ROS 2 uses a distributed computing architecture where different components (nodes) communicate through topics, services, and actions. This architecture enables:

- **Decoupling**: Components can be developed and tested independently
- **Scalability**: New components can be added without modifying existing ones
- **Robustness**: Failure in one component doesn't necessarily affect others
- **Flexibility**: Different programming languages can be used for different components

### Communication Patterns

1. **Topics (Publish/Subscribe)**: Asynchronous, many-to-many communication
   - Used for sensor data, robot state, etc.
   - Publishers send messages, subscribers receive them

2. **Services (Request/Response)**: Synchronous, one-to-one communication
   - Used for actions that require a response
   - Client sends request, server responds

3. **Actions**: Long-running tasks with feedback
   - Used for navigation, manipulation, etc.
   - Provides goal, feedback, and result

## Practical Implementation: Basic ROS 2 Package

Let's create our first ROS 2 package that will serve as the foundation for our robot's nervous system.

### Package Structure

```
ros_packages/robot_control/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── robot_control
├── test/
│   ├── __init__.py
│   └── test_copyright.py
├── robot_control/
│   ├── __init__.py
│   ├── sensor_publisher.py
│   ├── actuator_subscriber.py
│   └── communication_example.py
└── launch/
    └── communication_demo.launch.py
```

### Creating the Sensor Publisher

The sensor publisher will simulate sensor data from our humanoid robot, such as joint positions, IMU readings, and camera data.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import String
import random

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # Create publishers for different sensor types
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)

        # Timer for publishing sensor data
        self.timer = self.create_timer(0.1, self.publish_sensor_data)  # 10Hz

        self.joint_names = [
            'left_hip', 'left_knee', 'left_ankle',
            'right_hip', 'right_knee', 'right_ankle',
            'left_shoulder', 'left_elbow', 'left_wrist',
            'right_shoulder', 'right_elbow', 'right_wrist',
            'head_yaw', 'head_pitch'
        ]

        self.get_logger().info('Sensor publisher node started')

    def publish_sensor_data(self):
        # Publish joint states
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = [random.uniform(-1.5, 1.5) for _ in self.joint_names]
        joint_msg.velocity = [random.uniform(-0.5, 0.5) for _ in self.joint_names]
        joint_msg.effort = [random.uniform(-10.0, 10.0) for _ in self.joint_names]

        self.joint_pub.publish(joint_msg)

        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        # Simulate some random orientation
        imu_msg.orientation.x = random.uniform(-0.1, 0.1)
        imu_msg.orientation.y = random.uniform(-0.1, 0.1)
        imu_msg.orientation.z = random.uniform(-0.1, 0.1)
        imu_msg.orientation.w = 1.0  # Normalize

        self.imu_pub.publish(imu_msg)

        # Publish robot status
        status_msg = String()
        status_msg.data = 'Operational'
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()

    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating the Actuator Subscriber

The actuator subscriber will receive commands and simulate actuator responses:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import time

class ActuatorSubscriber(Node):
    def __init__(self):
        super().__init__('actuator_subscriber')

        # Create subscribers for different actuator commands
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray, 'joint_commands', self.joint_cmd_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Publisher for joint states (to verify actuator response)
        self.joint_state_pub = self.create_publisher(JointState, 'actuator_states', 10)

        self.get_logger().info('Actuator subscriber node started')

    def joint_cmd_callback(self, msg):
        self.get_logger().info(f'Received joint commands: {msg.data}')
        # Simulate actuator movement
        time.sleep(0.05)  # Simulate movement time

        # Publish updated joint states
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [f'joint_{i}' for i in range(len(msg.data))]
        joint_msg.position = list(msg.data)
        self.joint_state_pub.publish(joint_msg)

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'Received velocity command: linear={msg.linear}, angular={msg.angular}')
        # Simulate movement based on velocity command
        time.sleep(0.1)  # Simulate movement time

def main(args=None):
    rclpy.init(args=args)
    actuator_subscriber = ActuatorSubscriber()

    try:
        rclpy.spin(actuator_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        actuator_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch File Configuration

Let's create a launch file to run both nodes together:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # Sensor publisher node
    sensor_publisher = Node(
        package='robot_control',
        executable='sensor_publisher',
        name='sensor_publisher',
        output='screen'
    )

    # Actuator subscriber node
    actuator_subscriber = Node(
        package='robot_control',
        executable='actuator_subscriber',
        name='actuator_subscriber',
        output='screen'
    )

    # Add nodes to launch description
    ld.add_action(sensor_publisher)
    ld.add_action(actuator_subscriber)

    return ld
```

## Communication Patterns in Detail

### Publisher-Subscriber Pattern

The publisher-subscriber pattern is fundamental to ROS 2. Publishers send messages to topics, and subscribers receive messages from topics. This enables asynchronous communication between nodes.

```python
# Publisher example
publisher = self.create_publisher(String, 'topic_name', 10)
msg = String()
msg.data = 'Hello World'
publisher.publish(msg)

# Subscriber example
subscriber = self.create_subscription(String, 'topic_name', callback_function, 10)
```

### Service Client-Server Pattern

Services provide synchronous request-response communication:

```python
# Service server
self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    self.get_logger().info(f'Returning {response.sum}')
    return response

# Service client
self.cli = self.create_client(AddTwoInts, 'add_two_ints')
```

## Quality of Service (QoS) Settings

QoS settings determine how messages are delivered and handled:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# For sensor data (real-time, lose some messages is OK)
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)

# For critical commands (must be delivered)
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)
```

## Debugging Communication Issues

Common debugging techniques include:
- Using `ros2 topic list` and `ros2 topic echo` to monitor topics
- Using `ros2 node list` to see active nodes
- Using `ros2 run rqt_graph rqt_graph` to visualize the communication graph
- Checking node logs with `ros2 run rqt_console rqt_console`

## Practical Exercise

Create a simple communication system where:
1. A "brain" node publishes sensor requests
2. A "sensor" node responds with sensor data
3. An "actuator" node receives commands and publishes execution status

This exercise will help you understand the flow of information in a robot's nervous system.

## Summary

In this module, we've established the foundation for our robot's communication system. The ROS 2 nervous system enables different components to work together while maintaining modularity and scalability. In the next module, we'll build on this foundation by creating digital twin environments where our robot can operate safely and effectively.