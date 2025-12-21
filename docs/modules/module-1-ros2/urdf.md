---
sidebar_position: 4
---

# URDF: Unified Robot Description Format

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the physical and visual properties of robots, including links, joints, and their relationships.

## Introduction to URDF

URDF is essential for:
- Robot visualization in RViz
- Simulation in Gazebo
- Kinematic analysis
- Motion planning
- Robot state publishing

## Basic URDF Structure

A basic URDF file structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.2 0" rpy="0 0 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Links

Links represent rigid bodies in the robot. Each link can have multiple properties:

### Visual Properties
- **geometry**: Shape definition (box, cylinder, sphere, mesh)
- **material**: Color and texture information
- **origin**: Position and orientation relative to the link frame

### Collision Properties
- **geometry**: Collision shape (simplified from visual for performance)
- **origin**: Position and orientation relative to the link frame

### Inertial Properties
- **mass**: Mass of the link
- **inertia**: Inertia tensor values (ixx, ixy, ixz, iyy, iyz, izz)

### Example Link Definition

```xml
<link name="link_name">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.2 0.3"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>

  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.2 0.3"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
</link>
```

## Joints

Joints define the relationship between links and specify how they can move relative to each other.

### Joint Types

1. **revolute**: Rotational joint with limited range
2. **continuous**: Rotational joint without limits
3. **prismatic**: Linear sliding joint with limits
4. **fixed**: No movement (rigid connection)
5. **floating**: 6 DOF (not supported in Gazebo)
6. **planar**: Movement on a plane

### Joint Properties

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0.5 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

- **parent/child**: Links connected by the joint
- **origin**: Position and orientation of the joint
- **axis**: Axis of rotation or translation
- **limit**: Range of motion, effort, and velocity limits
- **dynamics**: Damping and friction coefficients

## Robot Base and World Frame

The first link in a URDF is typically the base link:

```xml
<link name="base_link">
  <!-- Base link definition -->
</link>
```

This link serves as the reference frame for the entire robot.

## Transmissions

Transmissions define how joints are connected to actuators:

```xml
<transmission name="wheel_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="wheel_joint">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="wheel_motor">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Gazebo Integration

Special Gazebo tags can be added to URDF files:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>

<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <left_joint>wheel_left_joint</left_joint>
    <right_joint>wheel_right_joint</right_joint>
  </plugin>
</gazebo>
```

## Xacro: XML Macros for URDF

Xacro allows parameterization and macros in URDF files:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>

  <xacro:macro name="wheel" params="prefix *origin">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

</robot>
```

## Common URDF Patterns

### Fixed Base Robot

```xml
<robot name="fixed_robot">
  <link name="base_link"/>
  <!-- Additional links and joints -->
</robot>
```

### Floating Base Robot

```xml
<robot name="mobile_robot">
  <link name="base_footprint"/>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.1"/>
  </joint>
  <link name="base_link"/>
  <!-- Additional links and joints -->
</robot>
```

## Validation and Visualization

### URDF Validation

```bash
check_urdf /path/to/robot.urdf
```

### Visualization

```bash
ros2 run rviz2 rviz2
```

## Best Practices

1. **Consistent Naming**: Use clear, descriptive names for links and joints
2. **Proper Mass Properties**: Accurate inertial properties for simulation
3. **Collision vs Visual**: Use simpler collision geometry for performance
4. **Joint Limits**: Always specify appropriate joint limits
5. **Origin Consistency**: Use consistent coordinate frames
6. **Parameterization**: Use Xacro for complex robots to avoid repetition

## Troubleshooting Common Issues

1. **Missing Joint Definitions**: Ensure all joints have parent and child links
2. **Invalid Inertial Properties**: Check that inertia values follow physical constraints
3. **Coordinate Frame Issues**: Verify origin transformations are correct
4. **Mesh Path Problems**: Ensure mesh files are in correct locations

URDF is fundamental to representing robots in ROS and simulation environments. Properly structured URDF files enable visualization, simulation, and motion planning for robotic systems.