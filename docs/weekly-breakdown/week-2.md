---
sidebar_position: 2
---

# Week 2: ROS 2 Advanced Concepts & Robot Modeling

## Learning Objectives
By the end of Week 2, students will be able to:
- Implement ROS 2 nodes using the rclpy Python client library
- Create and interpret URDF (Unified Robot Description Format) files
- Model simple robots with proper kinematic chains
- Integrate robot models with ROS 2 systems

## Day 1: rclpy Deep Dive
### Topics Covered
- Python client library architecture
- Node lifecycle management
- Publisher and subscriber implementation in Python
- Service and client implementation in Python

### Readings
- Module 1, Sections 3.1-3.3: rclpy Fundamentals
- ROS 2 Python client library documentation

### Activities
- Implement advanced publisher with custom message types
- Create subscriber with callback functions
- Develop service server and client in Python

## Day 2: Introduction to URDF
### Topics Covered
- URDF (Unified Robot Description Format) basics
- Links, joints, and their properties
- Visual and collision elements
- Inertial properties and dynamics

### Readings
- Module 1, Sections 4.1-4.2: URDF Fundamentals
- URDF specification and best practices

### Activities
- Create simple URDF model of a mobile robot
- Add visual and collision properties
- Validate URDF with check_urdf tool

## Day 3: Complex Robot Models
### Topics Covered
- Multi-link robot modeling
- Joint types and constraints
- Transmission elements
- Gazebo extensions in URDF

### Readings
- Module 1, Sections 4.3-4.4: Advanced URDF
- Gazebo-ROS integration in URDF

### Activities
- Model a 2-link manipulator
- Add different joint types (revolute, prismatic, continuous)
- Integrate with Gazebo simulation

## Day 4: Robot State Publishing
### Topics Covered
- Robot State Publisher
- Joint State Publisher
- TF (Transform) tree concepts
- URDF to TF broadcasting

### Readings
- Module 1, Sections 4.5-4.6: Robot State Management
- TF and robot state publisher documentation

### Activities
- Set up robot state publisher with URDF
- Publish joint states for robot model
- Visualize robot in RViz2

## Day 5: Week 2 Integration Lab
### Topics Covered
- Integration of rclpy with URDF models
- Robot simulation setup
- Troubleshooting robot models

### Activities
- Complete integrated robot system
- Test robot in simulation environment
- Debug common URDF issues
- Prepare for Week 3

## Assignments Due
- Assignment 2.1: Advanced rclpy Implementation (Due Day 2)
- Assignment 2.2: URDF Robot Model (Due Day 5)

## Assessment
- rclpy implementation: 25%
- URDF model creation and validation: 30%
- Integration with robot state publishing: 20%

## Resources
- [rclpy API Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [Robot State Publisher](http://wiki.ros.org/robot_state_publisher)
- [Week 2 Slides](#) (to be posted)

## Next Week Preview
Week 3 introduces simulation environments with Gazebo and Unity. Students will learn to create simulated environments and test robot behaviors in safe, repeatable conditions.