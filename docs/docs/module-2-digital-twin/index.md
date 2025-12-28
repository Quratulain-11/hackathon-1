---
sidebar_position: 1
title: "Digital Twin: Physics-Accurate Simulation"
---

# Module 2: Digital Twin - Physics-Accurate Simulation

## Creating Realistic Robot Environments

In this module, we'll build digital twin environments that accurately simulate real-world physics and robot behavior. A digital twin is a virtual replica of a physical system that allows us to test, validate, and optimize our robot's performance before deploying it in the real world.

## Learning Objectives

By the end of this module, you will:
- Understand the principles of physics simulation for robotics
- Create URDF (Unified Robot Description Format) models for humanoid robots
- Design Gazebo simulation environments with realistic physics
- Configure sensors for accurate perception in simulation
- Implement physics properties that mirror real-world behavior
- Debug simulation discrepancies with real-world performance

## Understanding Digital Twins in Robotics

A digital twin in robotics serves multiple critical functions:
- **Development**: Test algorithms without physical hardware
- **Validation**: Verify safety and performance before deployment
- **Optimization**: Fine-tune parameters in a controlled environment
- **Training**: Generate synthetic data for AI models
- **Troubleshooting**: Reproduce and debug issues in simulation

## URDF: The Robot Description Language

URDF (Unified Robot Description Format) is XML-based and defines the physical structure of our robot. It includes links (rigid bodies), joints (connections), and inertial properties.

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Example joint and link -->
  <joint name="hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="0.0 0.1 0.0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.01"/>
  </joint>

  <link name="left_hip">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
</robot>
```

### Key URDF Components

1. **Links**: Rigid bodies with visual, collision, and inertial properties
2. **Joints**: Connections between links with specific degrees of freedom
3. **Materials**: Visual appearance properties
4. **Transmissions**: Motor and actuator interfaces
5. **Gazebo plugins**: Simulation-specific extensions

## Gazebo Simulation Environment

Gazebo provides a physics engine (ODE, Bullet, or DART) that simulates realistic robot-world interactions.

### World File Structure

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_world">
    <!-- Include the robot -->
    <include>
      <uri>model://humanoid_robot</uri>
      <pose>0 0 1.0 0 0 0</pose>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Objects for interaction -->
    <model name="table">
      <pose>2 0 0 0 0 0</pose>
      <link name="table_link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Physics engine configuration -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
  </world>
</sdf>
```

## Physics Configuration

### Gravity and Time Settings

```yaml
# simulation/config/physics.yaml
physics_engine:
  type: "ode"  # Options: ode, bullet, dart
  max_step_size: 0.001  # Simulation time step (seconds)
  real_time_factor: 1.0  # Simulation speed relative to real time
  real_time_update_rate: 1000  # Updates per second
  gravity: [0, 0, -9.8]  # Gravity vector [x, y, z]
```

### Collision Detection Parameters

```yaml
collision_detection:
  contact_surface_layer: 0.001  # Contact layer thickness (meters)
  constraint_sor: 1.3  # Successive Over-Relaxation parameter
  constraint_iterations: 50  # Constraint solver iterations
  constraint_tolerance: 0.001  # Constraint error tolerance
```

## Sensor Simulation

### Camera Sensor Configuration

```xml
<sensor name="camera" type="camera">
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <camera name="head_camera">
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <frame_name>camera_frame</frame_name>
    <min_depth>0.1</min_depth>
    <max_depth>10.0</max_depth>
  </plugin>
</sensor>
```

### IMU Sensor Configuration

```xml
<sensor name="imu" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

## Simulation Launch Configuration

### Robot Spawn Launch File

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='humanoid_world.sdf')

    # Package names
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_simulation = FindPackageShare('simulation')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('simulation'),
                'worlds',
                world
            ]),
            'verbose': 'false',
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command([
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([
                    FindPackageShare("simulation"),
                    "urdf",
                    "humanoid_robot.urdf.xacro"
                ])
            ])
        }]
    )

    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='humanoid_world.sdf',
            description='Choose one of the world files from `/simulation/worlds`'
        ),
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
```

## Physics Accuracy Considerations

### Mass and Inertia Properties

Accurate mass and inertia properties are crucial for realistic simulation:

```xml
<inertial>
  <!-- Mass in kilograms -->
  <mass value="1.5"/>

  <!-- Inertia tensor (diagonal elements for box-like objects) -->
  <inertia
    ixx="0.001" ixy="0.0" ixz="0.0"
    iyy="0.002" iyz="0.0"
    izz="0.003"/>
</inertial>
```

### Friction and Damping

```xml
<joint name="knee_joint" type="revolute">
  <parent link="thigh"/>
  <child link="shin"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
  <!-- Joint dynamics -->
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

## Sensor Noise and Realism

Adding realistic noise to sensors helps bridge the sim-to-real gap:

```xml
<!-- In URDF file -->
<gazebo reference="camera_link">
  <sensor type="camera" name="camera">
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <!-- Add realistic noise -->
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </plugin>
  </sensor>
</gazebo>
```

## Debugging Simulation Issues

Common simulation problems and solutions:

1. **Robot falls through the ground**: Check collision geometries and mass properties
2. **Jittery movement**: Reduce time step or adjust solver parameters
3. **Unrealistic behavior**: Verify mass, inertia, and friction values
4. **Performance issues**: Simplify collision geometries or reduce update rates

## Practical Exercise

Create a simple simulation environment with:
1. A humanoid robot model with at least 10 joints
2. A world with obstacles and objects to interact with
3. Camera and IMU sensors with realistic noise models
4. Physics parameters that match real-world expectations

## Summary

The digital twin provides a safe, cost-effective environment to develop and test our humanoid robot. By creating physics-accurate simulations, we can validate our control algorithms, test safety systems, and optimize performance before deploying on real hardware. In the next module, we'll explore how to give our robot an "AI brain" that can process sensor data and make intelligent decisions.