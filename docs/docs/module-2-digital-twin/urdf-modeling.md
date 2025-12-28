---
sidebar_position: 2
title: "URDF and Robot Modeling"
---

# URDF and Robot Modeling

## Creating Detailed Robot Models

URDF (Unified Robot Description Format) is the foundation of robot simulation in ROS 2. In this section, we'll explore advanced URDF concepts and create detailed models of our humanoid robot.

## Advanced URDF Concepts

### Xacro for Complex Models

Xacro (XML Macros) allows us to create parameterized and reusable URDF models:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="robot_name" value="humanoid" />

  <!-- Materials -->
  <xacro:macro name="material_blue">
    <material name="blue">
      <color rgba="0.0 0.0 0.8 1.0"/>
    </material>
  </xacro:macro>

  <!-- Inertial macro -->
  <xacro:macro name="inertial_sphere" params="mass radius *origin">
    <inertial>
      <mass value="${mass}" />
      <xacro:insert_block name="origin" />
      <inertia ixx="${0.4 * mass * radius * radius}" ixy="0.0" ixz="0.0"
               iyy="${0.4 * mass * radius * radius}" iyz="0.0"
               izz="${0.4 * mass * radius * radius}" />
    </inertial>
  </xacro:macro>

  <!-- Joint macro -->
  <xacro:macro name="simple_joint" params="name type parent child axis xyz rpy lower upper effort velocity">
    <joint name="${name}" type="${type}">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="${axis}"/>
      <limit lower="${lower}" upper="${upper}" effort="${effort}" velocity="${velocity}"/>
      <dynamics damping="0.1" friction="0.01"/>
    </joint>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <!-- Torso -->
  <xacro:simple_joint name="torso_joint" type="fixed"
                      parent="base_link" child="torso"
                      axis="0 0 0" xyz="0 0 0.1" rpy="0 0 0"
                      lower="0" upper="0" effort="0" velocity="0"/>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.25"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <xacro:simple_joint name="head_joint" type="revolute"
                      parent="torso" child="head"
                      axis="0 0 1" xyz="0 0 0.5" rpy="0 0 0"
                      lower="${-M_PI/2}" upper="${M_PI/2}" effort="10.0" velocity="1.0"/>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <xacro:inertial_sphere mass="2.0" radius="0.1">
      <origin xyz="0 0 0"/>
    </xacro:inertial_sphere>
  </link>

  <!-- Left Arm -->
  <xacro:simple_joint name="left_shoulder_pitch" type="revolute"
                      parent="torso" child="left_upper_arm"
                      axis="0 1 0" xyz="0.2 0 0.3" rpy="0 0 0"
                      lower="${-M_PI/2}" upper="${M_PI/2}" effort="50.0" velocity="2.0"/>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.15"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <xacro:simple_joint name="left_elbow" type="revolute"
                      parent="left_upper_arm" child="left_lower_arm"
                      axis="0 1 0" xyz="0 0 0.3" rpy="0 0 0"
                      lower="${-M_PI/2}" upper="${M_PI/2}" effort="30.0" velocity="2.0"/>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 0.125"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <xacro:simple_joint name="left_hip" type="revolute"
                      parent="torso" child="left_thigh"
                      axis="0 0 1" xyz="0.1 0 0" rpy="0 0 0"
                      lower="${-M_PI/2}" upper="${M_PI/2}" effort="100.0" velocity="1.0"/>

  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <xacro:simple_joint name="left_knee" type="revolute"
                      parent="left_thigh" child="left_shin"
                      axis="0 0 1" xyz="0 0 -0.4" rpy="0 0 0"
                      lower="${-M_PI/2}" upper="0" effort="80.0" velocity="1.0"/>

  <link name="left_shin">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.5"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.0008"/>
    </inertial>
  </link>

  <!-- Right side (similar to left, mirrored) -->
  <xacro:simple_joint name="right_hip" type="revolute"
                      parent="torso" child="right_thigh"
                      axis="0 0 1" xyz="-0.1 0 0" rpy="0 0 0"
                      lower="${-M_PI/2}" upper="${M_PI/2}" effort="100.0" velocity="1.0"/>

  <link name="right_thigh">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <xacro:simple_joint name="right_knee" type="revolute"
                      parent="right_thigh" child="right_shin"
                      axis="0 0 1" xyz="0 0 -0.4" rpy="0 0 0"
                      lower="${-M_PI/2}" upper="0" effort="80.0" velocity="1.0"/>

  <link name="right_shin">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.5"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.0008"/>
    </inertial>
  </link>

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid_robot</robotNamespace>
    </plugin>
  </gazebo>

  <!-- Camera on head -->
  <joint name="head_camera_joint" type="fixed">
    <parent link="head"/>
    <child link="head_camera_frame"/>
    <origin xyz="0.05 0 0" rpy="0 0 0"/>
  </joint>

  <link name="head_camera_frame"/>

  <gazebo reference="head_camera_frame">
    <sensor type="camera" name="head_camera">
      <update_rate>30</update_rate>
      <camera name="head">
        <horizontal_fov>1.047</horizontal_fov>
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
        <frame_name>head_camera_frame</frame_name>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

## Gazebo Integration

### Transmission Configuration

To enable joint control in simulation, we need to define transmissions:

```xml
<!-- In URDF file -->
<xacro:macro name="transmission_block" params="joint_name">
  <transmission name="${joint_name}_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="${joint_name}">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="${joint_name}_motor">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
</xacro:macro>

<!-- Use the macro for each joint -->
<xacro:transmission_block joint_name="left_shoulder_pitch"/>
<xacro:transmission_block joint_name="left_elbow"/>
<xacro:transmission_block joint_name="right_hip"/>
<xacro:transmission_block joint_name="right_knee"/>
<!-- Add for all joints that need control -->
```

### Gazebo-Specific Properties

```xml
<!-- Add to links that need special Gazebo properties -->
<gazebo reference="left_upper_arm">
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
  <material>Gazebo/Blue</material>
  <self_collide>false</self_collide>
  <gravity>true</gravity>
</gazebo>
```

## Model Validation

### Checking URDF Syntax

```bash
# Validate URDF syntax
check_urdf /path/to/robot.urdf

# Or with xacro
xacro /path/to/robot.urdf.xacro > /tmp/robot.urdf && check_urdf /tmp/robot.urdf
```

### Visualizing in RViz

```bash
# Launch robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(xacro /path/to/robot.urdf.xacro)

# Then launch RViz to visualize
ros2 run rviz2 rviz2
```

## Advanced Modeling Techniques

### Flexible Joint Modeling

For more realistic joint behavior, we can model joint compliance:

```xml
<joint name="flexible_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
  <!-- Flexible joint with spring-like behavior -->
  <dynamics damping="1.0" friction="0.1" spring_stiffness="1000" spring_reference="0"/>
</joint>
```

### Multi-body Simulation Considerations

For complex robots with many degrees of freedom:

```xml
<gazebo>
  <!-- Physics parameters for the entire model -->
  <static>false</static>
  <self_collide>false</self_collide>
  <enable_wind>false</enable_wind>
  <kinematic>false</kinematic>
  <gravity>true</gravity>

  <!-- Performance optimization -->
  <max_contacts>10</max_contacts>
</gazebo>
```

## Model Optimization

### Simplified Collision Geometries

For better performance, use simplified collision geometries:

```xml
<!-- Complex visual geometry -->
<link name="complex_link">
  <visual>
    <geometry>
      <mesh filename="meshes/complex_shape.stl"/>
    </geometry>
  </visual>

  <!-- Simplified collision geometry -->
  <collision>
    <geometry>
      <box size="0.2 0.1 0.3"/>  <!-- Approximate bounding box -->
    </geometry>
  </collision>
</link>
```

### Level of Detail (LOD)

For large environments, consider different levels of detail:

```xml
<visual>
  <geometry>
    <mesh filename="meshes/robot_high_detail.dae"/>
  </geometry>
  <!-- LOD for performance -->
  <material name="lod_material">
    <script>
      <name>robot_material</name>
      <file>materials/scripts/Robot.material</file>
    </script>
  </material>
</visual>
```

## Troubleshooting Common Issues

### Joint Limit Problems

```xml
<!-- Make sure limits are appropriate -->
<limit lower="-1.57" upper="1.57" effort="100.0" velocity="2.0"/>
<!-- For continuous joints, don't set limits -->
<joint name="continuous_joint" type="continuous">
  <axis xyz="0 0 1"/>
  <!-- No limit element for continuous joints -->
</joint>
```

### Inertia Calculation

Use proper tools to calculate inertia for complex shapes:

```xml
<!-- For box: Ixx = 1/12 * m * (h² + d²) -->
<inertia ixx="0.0833" ixy="0.0" ixz="0.0"
         iyy="0.0833" iyz="0.0"
         izz="0.0833"/>
```

## Practical Exercise

Create a complete humanoid robot model with:
1. At least 12 joints (6 for legs, 4 for arms, 2 for head/waist)
2. Proper mass and inertia properties for each link
3. Xacro macros for reusability
4. Gazebo plugins for simulation
5. Sensor mounts for camera and IMU

## Summary

Creating accurate URDF models is crucial for successful simulation. The model must balance realism with computational efficiency while maintaining the physical properties necessary for realistic behavior. In the next section, we'll explore how to create realistic simulation environments and configure physics parameters for accurate robot-world interactions.