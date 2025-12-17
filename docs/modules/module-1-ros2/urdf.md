---
sidebar_position: 4
---

# URDF: Unified Robot Description Format

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including its links, joints, and other components. URDF is essential for simulation, visualization, and understanding robot kinematics.

## URDF Structure Overview

A URDF file is an XML document that describes a robot's structure using the following main elements:

- **Links**: Rigid bodies of the robot (e.g., chassis, wheels, arms)
- **Joints**: Connections between links (e.g., revolute, prismatic, fixed)
- **Visual**: How the robot appears in simulation and visualization
- **Collision**: How the robot interacts with the physical environment
- **Inertial**: Physical properties like mass and moments of inertia

## Basic URDF Example

Here's a simple URDF file for a differential drive robot:

```xml
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Joints connecting wheels to base -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.2 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.2 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

## Link Elements

Links represent rigid bodies in the robot. Each link can have multiple properties:

### Visual Properties
- **geometry**: Defines the shape (box, cylinder, sphere, mesh)
- **material**: Defines color and appearance
- **origin**: Position and orientation relative to the link frame

### Collision Properties
- **geometry**: Defines the collision shape (usually simpler than visual)
- **origin**: Position and orientation relative to the link frame

### Inertial Properties
- **mass**: Mass of the link in kilograms
- **inertia**: Moments of inertia matrix values

## Joint Elements

Joints connect links and define how they can move relative to each other:

### Joint Types
- **fixed**: No movement between links
- **continuous**: Continuous rotation (like a wheel)
- **revolute**: Limited rotation (like an elbow)
- **prismatic**: Linear sliding motion
- **planar**: Motion on a plane
- **floating**: 6 DOF movement

### Joint Properties
- **parent**: The parent link in the kinematic chain
- **child**: The child link in the kinematic chain
- **origin**: Position and orientation of the joint
- **axis**: Axis of rotation or translation
- **limits**: For revolute joints (lower, upper, effort, velocity)

## Advanced URDF Features

### Materials
Materials define the appearance of links:

```xml
<material name="red">
  <color rgba="1 0 0 1"/>
</material>

<material name="green">
  <color rgba="0 1 0 1"/>
</material>

<material name="blue">
  <color rgba="0 0 1 1"/>
</material>
```

### Gazebo Extensions
URDF can be extended with Gazebo-specific properties:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <kp>1000000.0</kp>
  <kd>1.0</kd>
</gazebo>
```

### Transmission Elements
Define how actuators connect to joints:

```xml
<transmission name="left_wheel_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_wheel_joint">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_wheel_motor">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## URDF Best Practices

### 1. Use Proper Reference Frames
Always define clear reference frames for your robot, typically with the base_link at the robot's center of mass or geometric center.

### 2. Separate Visual and Collision Models
Use simpler geometries for collision detection to improve performance:

```xml
<!-- Visual: Detailed model -->
<visual>
  <mesh filename="meshes/detailed_robot.dae"/>
</visual>

<!-- Collision: Simplified model -->
<collision>
  <geometry>
    <box size="0.5 0.3 0.2"/>
  </geometry>
</collision>
```

### 3. Accurate Inertial Properties
Proper inertial properties are crucial for realistic simulation:

```xml
<inertial>
  <mass value="1.0"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="0.083" ixy="0.0" ixz="0.0"
           iyy="0.083" iyz="0.0" izz="0.083"/>
</inertial>
```

### 4. Use Xacro for Complex Models
Xacro is an XML macro language that makes URDF more readable and maintainable:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_with_xacro">

  <!-- Define properties -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>

  <!-- Define a macro for wheels -->
  <xacro:macro name="wheel" params="prefix parent x y z">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.2"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0"
                 iyy="0.001" iyz="0.0" izz="0.002"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${x} ${y} ${z}" rpy="1.5708 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:wheel prefix="left" parent="base_link" x="0" y="0.2" z="-0.05"/>
  <xacro:wheel prefix="right" parent="base_link" x="0" y="-0.2" z="-0.05"/>
</robot>
```

## URDF Tools and Validation

### Checking URDF Files
Use the `check_urdf` tool to validate your URDF:

```bash
check_urdf /path/to/robot.urdf
```

### Visualizing URDF
Use `rviz2` to visualize your robot model:

```bash
ros2 run rviz2 rviz2
```

### Generating Joint Information
The `check_urdf` tool also shows joint information:

```bash
# Output joint information
check_urdf /path/to/robot.urdf
```

## Practical Exercise: Create a Simple Robot

Create a URDF file for a simple 2-link manipulator with:
1. A base link
2. Two arm links connected by revolute joints
3. Proper visual and collision properties
4. Accurate inertial properties

This exercise will help you understand the relationship between links and joints in URDF.

## Common URDF Issues and Solutions

### 1. Floating Point Precision
Use appropriate precision for URDF values to avoid numerical issues.

### 2. Inconsistent Units
Always use consistent units (meters for length, kilograms for mass).

### 3. Invalid Inertial Matrices
Ensure inertia matrices are physically valid (positive definite).

### 4. Kinematic Loops
URDF doesn't support kinematic loops; use other tools for closed-chain mechanisms.

## Summary

URDF is a fundamental tool for describing robot models in ROS:

- Links define the rigid bodies of the robot
- Joints define how links connect and move
- Visual and collision properties enable simulation and visualization
- Inertial properties enable physics simulation
- Xacro simplifies complex robot descriptions
- Proper validation ensures correct robot behavior

Understanding URDF is essential for creating robots that can be simulated, visualized, and controlled effectively in ROS.