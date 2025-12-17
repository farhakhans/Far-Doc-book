---
sidebar_position: 3
---

# Edge Kit: Advanced Mobile Robot Platform

## Overview

The Edge Kit represents a mid-tier mobile robot platform designed for advanced Physical AI learning and research. This platform includes comprehensive perception capabilities, powerful edge computing, and robust mechanical design suitable for complex navigation and manipulation tasks.

## System Architecture

### Core Components
| Component | Specification | Purpose |
|-----------|---------------|---------|
| **Main Computer** | NVIDIA Jetson Orin AGX (64GB) | Primary processing and AI acceleration |
| **Robot Chassis** | Custom differential drive with 4WD option | Mobile platform base |
| **Power System** | 48V 20Ah smart battery with BMS | Power management and safety |
| **Motor Controllers** | 4x 100A ESC with feedback | Precise motor control |
| **Chassis Controller** | STM32-based with CAN interface | Low-level motion control |

### Perception Suite
| Sensor | Model/Type | Purpose | Interface |
|--------|------------|---------|-----------|
| **Main Camera** | Global shutter RGB (120° FOV) | Navigation and object detection | MIPI CSI-2 |
| **Depth Camera** | Intel RealSense D455 or equivalent | 3D perception and mapping | USB 3.0 |
| **LIDAR** | 360° 2D LIDAR (10m range) | Navigation and obstacle detection | USB/UART |
| **IMU** | 9-axis with magnetometer | Orientation and motion sensing | I2C/SPI |
| **Additional Cameras** | 2x fisheye (180° FOV) | Wide-area monitoring | MIPI CSI-2 |

### Specifications Summary
| Category | Specification | Value |
|----------|---------------|-------|
| **Dimensions** | Length x Width x Height | 50cm x 40cm x 30cm |
| **Weight** | Total system weight | 15-20kg |
| **Speed** | Maximum linear speed | 1.5 m/s |
| **Payload** | Maximum payload capacity | 5kg |
| **Runtime** | Operating time on full charge | 4-6 hours |
| **Range** | Wireless communication range | 100m+ |

## Mechanical Design

### Chassis Construction
- **Frame**: Aluminum extrusion construction with mounting points
- **Wheels**: 20cm diameter with rubber treads for traction
- **Suspension**: Independent wheel suspension for uneven terrain
- **Ground Clearance**: 8cm minimum for obstacle navigation
- **Modularity**: Standard mounting points for custom attachments

### Manipulation Options
| Option | Description | DOF | Payload | Price |
|--------|-------------|-----|---------|-------|
| **Basic Gripper** | Parallel jaw gripper | 1 | 1kg | $300-500 |
| **5-DOF Arm** | Compact robotic arm | 5 | 0.5kg | $1,500-2,500 |
| **7-DOF Arm** | Advanced manipulation arm | 7 | 1kg | $3,000-5,000 |
| **Custom End-Effector** | Application-specific | Variable | Variable | $500-2,000 |

### Mobility Features
- **Differential Drive**: Standard configuration
- **4WD Option**: Enhanced mobility and traction
- **Omni-wheel Kit**: Holonomic movement capability
- **Track Kit**: For rough terrain applications
- **Lifting Mechanism**: For object manipulation at different heights

## Computing Platform

### NVIDIA Jetson Orin AGX Specifications
| Component | Specification | Notes |
|-----------|---------------|-------|
| **CPU** | 12-core ARM v8.4 | 2.2 GHz per core |
| **GPU** | 2048-core NVIDIA Ampere | 825 MHz |
| **DL Accelerator** | 2x NVDLA | Deep learning acceleration |
| **Memory** | 64GB 256-bit LPDDR5 | 204.8 GB/s bandwidth |
| **Storage** | 64GB eMMC + M.2 NVMe | Expandable storage |
| **Connectivity** | WiFi 6, Bluetooth 5.2, Ethernet | Multiple interfaces |

### Edge AI Capabilities
- **Tensor Processing**: 275 TOPS INT8, 137 TOPS INT4
- **Video Processing**: 4x 4K60 or 8x 4K30 encode/decode
- **Camera Inputs**: 16x MIPI CSI-2 lanes
- **Real-time AI**: Simultaneous perception and action

### Power Management
| Component | Power Consumption | Voltage | Notes |
|-----------|------------------|---------|-------|
| **Jetson Orin AGX** | 15-60W | 19V | Variable based on load |
| **LIDAR** | 5-8W | 12V | Continuous operation |
| **Cameras** | 3-5W | 5V | Multiple cameras |
| **Motors** | 200-500W | 48V | Peak during acceleration |
| **Peripherals** | 10-20W | Various | IMU, sensors, etc. |

## Sensor Integration

### Camera System
| Camera | Purpose | Resolution | FPS | FOV | Interface |
|--------|---------|------------|-----|-----|-----------|
| **Front RGB** | Navigation, object detection | 1280x720 | 60 | 120° | MIPI CSI-2 |
| **Front Depth** | 3D mapping, obstacle detection | 1280x720 | 30 | 86° H, 56° V | USB 3.0 |
| **Panoramic** | Environment monitoring | 640x480 | 30 | 180° | MIPI CSI-2 |
| **Bottom** | Ground inspection | 640x480 | 30 | 120° | MIPI CSI-2 |

### LIDAR Specifications
| Parameter | Value | Notes |
|-----------|-------|-------|
| **Range** | 0.15m - 10m | Effective range |
| **Accuracy** | ±3cm | At 10m distance |
| **Angular Resolution** | 0.1° - 0.4° | Configurable |
| **Scan Rate** | 5-15 Hz | Variable for power/performance |
| **Points per Revolution** | 360-1080 | Configurable density |

### IMU Capabilities
| Sensor | Range | Resolution | Bandwidth |
|--------|-------|------------|-----------|
| **Accelerometer** | ±16g | 0.488 mg | 1.1 kHz |
| **Gyroscope** | ±2000 dps | 70 mdps | 1.1 kHz |
| **Magnetometer** | ±1300 µT | 0.6 µT | 100 Hz |
| **Barometer** | 260-1260 hPa | 0.25 Pa | 75 Hz |

## Software Stack

### Pre-installed Software
| Component | Version | Purpose |
|-----------|---------|---------|
| **ROS 2** | Humble Hawksbill | Robot middleware |
| **Isaac ROS** | Latest | GPU-accelerated perception |
| **Isaac Sim** | Latest | Simulation environment |
| **Navigation2** | Latest | Path planning and navigation |
| **OpenCV** | Latest | Computer vision |
| **PyTorch/TensorFlow** | Latest | AI framework |

### Custom Software Components
- **Robot Drivers**: Low-level hardware interfaces
- **Sensor Fusion**: Multi-sensor data integration
- **Perception Pipeline**: Object detection and tracking
- **Navigation Stack**: Path planning and obstacle avoidance
- **Manipulation Interface**: Grasping and manipulation
- **Communication Layer**: Remote operation and monitoring

## Performance Benchmarks

### Computational Performance
| Task | Performance | Notes |
|------|-------------|-------|
| **Object Detection** | 30+ FPS | YOLOv8, 640x640 input |
| **SLAM** | Real-time | Cartographer or RTAB-Map |
| **Path Planning** | < 100ms | Global and local planners |
| **Manipulation** | 100+ Hz | Joint control loop |
| **Sensor Processing** | Real-time | All sensors simultaneously |

### Navigation Performance
| Metric | Target | Achieved |
|--------|--------|---------|
| **Localization Accuracy** | < 5cm | < 2cm (with good features) |
| **Navigation Success Rate** | > 90% | > 95% (structured environments) |
| **Path Efficiency** | < 1.2x optimal | 1.1x optimal |
| **Obstacle Avoidance** | 100% | 100% (for detectable obstacles) |

## Educational Applications

### Course Integration
- **ROS 2 Fundamentals**: Basic node creation and communication
- **Perception**: Object detection, SLAM, sensor fusion
- **Navigation**: Path planning, obstacle avoidance, localization
- **Manipulation**: Grasping, pick-and-place operations
- **AI Integration**: Deep learning for robotics applications

### Project Examples
1. **Autonomous Navigation**: Navigate unknown environments
2. **Object Recognition**: Identify and classify objects
3. **Manipulation Tasks**: Pick and place operations
4. **Human-Robot Interaction**: Voice and gesture recognition
5. **Multi-Robot Systems**: Coordination and communication

## Cost Analysis

### Base Configuration
| Component | Cost Range | Notes |
|-----------|------------|-------|
| **Chassis & Mechanics** | $1,500-2,500 | Custom fabrication |
| **Jetson Orin AGX** | $1,000-1,200 | 64GB version |
| **Sensors** | $1,000-1,500 | Camera, LIDAR, IMU |
| **Power System** | $300-500 | Battery and BMS |
| **Motors & Drivers** | $800-1,200 | High-quality components |
| **Electronics** | $500-800 | Controllers, interfaces |
| **Total Base Kit** | $5,100-7,200 | Without manipulator |

### Optional Upgrades
| Upgrade | Cost | Purpose |
|---------|------|---------|
| **Robotic Arm** | $1,500-5,000 | Manipulation capabilities |
| **4WD Conversion** | $800-1,200 | Enhanced mobility |
| **Additional Sensors** | $500-1,500 | Specialized perception |
| **Extended Battery** | $300-600 | Longer operation time |
| **Custom End-Effector** | $300-2,000 | Specialized manipulation |

## Comparison with Alternatives

### vs. Economy Kit
| Aspect | Edge Kit | Economy Kit |
|--------|----------|-------------|
| **Price** | $5,100+ | $500-1,500 |
| **Performance** | High | Basic |
| **Sensors** | Comprehensive | Minimal |
| **AI Capabilities** | Advanced | Basic |
| **Applications** | Research/Advanced | Learning/Basic |

### vs. Humanoid Platform
| Aspect | Edge Kit | Humanoid Platform |
|--------|----------|-------------------|
| **Price** | $5,100+ | $25,000+ |
| **Mobility** | Ground-based | Bipedal |
| **Complexity** | Moderate | High |
| **Research Focus** | Navigation, perception | Locomotion, interaction |
| **Development Time** | 1-3 months | 6-12 months |

## Maintenance and Support

### Expected Lifespan
- **Mechanical Components**: 5-7 years with maintenance
- **Electronics**: 3-5 years (obsolescence factor)
- **Battery**: 2-3 years (cycle life)
- **Software**: Continuous updates available

### Support Resources
- **Documentation**: Comprehensive user manuals
- **Tutorials**: Step-by-step guides for common tasks
- **Community**: Active user forum and support
- **Training**: Workshops and online courses
- **Warranty**: 1-2 year hardware warranty

## Purchase Recommendations

### For Individual Researchers
- Start with base configuration
- Add manipulator based on research needs
- Consider 4WD for outdoor applications

### For Educational Institutions
- Purchase 3-5 units for lab setup
- Standardize on one configuration
- Budget for spare parts and maintenance
- Consider extended warranties

### For Research Labs
- Multiple configurations for different research areas
- Redundant systems for reliability
- Advanced sensors for specialized research
- Integration with existing infrastructure

The Edge Kit provides an excellent balance of performance, capability, and cost for advanced Physical AI and humanoid robotics research and education.