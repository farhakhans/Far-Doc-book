---
sidebar_position: 1
---

# Hardware Requirements Overview

## Introduction

This document outlines the hardware requirements for implementing the Physical AI & Humanoid Robotics curriculum. The requirements are structured to accommodate different budget levels and educational contexts, from economy setups for introductory courses to full humanoid platforms for advanced research.

## Hardware Categories

### 1. Workstation Requirements
- **Development workstations** for simulation and programming
- **Edge computing platforms** for robot control
- **Cloud computing** alternatives for resource-intensive tasks

### 2. Robot Platforms
- **Economy kit**: Basic mobile robot for ROS 2 and simulation learning
- **Edge kit**: Advanced mobile robot with perception capabilities
- **Robot lab options**: Multiple robots for laboratory settings
- **Humanoid platforms**: Full humanoid robots for advanced research

### 3. Sensor and Actuator Requirements
- **Vision systems**: Cameras, depth sensors, and processing units
- **Navigation sensors**: LIDAR, IMU, GPS, and encoders
- **Manipulation systems**: Grippers, arms, and end-effectors

## Architecture Summary

| Component | Economy Kit | Edge Kit | Robot Lab | Humanoid Platform |
|-----------|-------------|----------|-----------|-------------------|
| Compute Platform | Raspberry Pi 4 | NVIDIA Jetson Orin | Multiple Jetson Orin | Humanoid Computer |
| Vision System | 2x RGB cameras | RGB-D camera + IMU | Multiple RGB-D + LIDAR | Stereo cameras + depth |
| Navigation | Wheel encoders | LIDAR + IMU | Multi-sensor fusion | Full-body perception |
| Manipulation | Basic gripper | 5-DOF arm | Multiple arms | Humanoid hands |
| Connectivity | WiFi | WiFi + Ethernet | Mesh network | Integrated |
| Power System | Battery pack | Intelligent battery | Central power | Integrated |

## Budget Considerations

### Cost Ranges
- **Economy Kit**: $500-1,500 per unit
- **Edge Kit**: $2,000-5,000 per unit
- **Robot Lab**: $10,000-50,000 total lab setup
- **Humanoid Platform**: $25,000-100,000+ per unit

### Total Cost of Ownership
- Initial hardware costs
- Maintenance and replacement
- Software licensing (if applicable)
- Training and support costs

## Educational Scalability

The hardware requirements are designed to scale from individual student projects to institutional robotics labs, allowing for:
- Progressive learning with increasing complexity
- Cost-effective implementation at different educational levels
- Upgrade paths as resources allow
- Shared resources for multiple students