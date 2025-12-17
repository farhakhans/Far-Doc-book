---
sidebar_position: 5
---

# Economy Kit: Basic Mobile Robot Platform

## Overview

The Economy Kit represents an entry-level mobile robot platform designed for introductory Physical AI learning. This cost-effective solution enables students to learn ROS 2 fundamentals, basic navigation, and simple perception tasks without significant financial investment.

## System Architecture

### Core Components

<div class="hardware-table">

| Component | Specification | Purpose |
|-----------|---------------|---------|
| **Main Computer** | Raspberry Pi 4 (8GB RAM) | Primary processing |
| **Robot Chassis** | 3D-printed differential drive | Mobile platform base |
| **Power System** | 11.1V 5000mAh LiPo with BMS | Power management |
| **Motor Controllers** | 2x 30A ESC with feedback | Motor control |
| **Chassis Controller** | Arduino Nano or ESP32 | Low-level motion control |

</div>

### Basic Sensor Suite

<div class="hardware-table">

| Sensor | Model/Type | Purpose | Interface |
|--------|------------|---------|-----------|
| **Camera** | Raspberry Pi Camera v3 | Navigation and object detection | CSI |
| **Distance Sensors** | 4x Ultrasonic (HC-SR04) | Obstacle detection | GPIO |
| **IMU** | 9-axis (MPU-9250) | Orientation and motion sensing | I2C |
| **Encoders** | Wheel encoders (built-in) | Odometry | GPIO |

</div>

### Specifications Summary

<div class="hardware-table">

| Category | Specification | Value |
|----------|---------------|-------|
| **Dimensions** | Length x Width x Height | 30cm x 25cm x 15cm |
| **Weight** | Total system weight | 3-5kg |
| **Speed** | Maximum linear speed | 0.5 m/s |
| **Payload** | Maximum payload capacity | 1kg |
| **Runtime** | Operating time on full charge | 2-3 hours |
| **Range** | Wireless communication range | 50m+ |

</div>

## Mechanical Design

### Chassis Construction
- **Frame**: 3D-printed ABS plastic with aluminum standoff supports
- **Wheels**: 10cm diameter with rubber treads
- **Ground Clearance**: 3cm for basic obstacle navigation
- **Mounting Points**: Standard 2020 aluminum extrusion compatible
- **Expandability**: Modular design for sensor additions

### Mobility Features
- **Differential Drive**: Standard 2-wheel configuration
- **Caster Wheel**: 3rd point of contact for stability
- **Modular Design**: Easy component replacement
- **Tool-free Assembly**: Quick-release mechanisms

### Expansion Options
| Option | Description | Cost | Purpose |
|--------|-------------|------|---------|
| **Additional Sensors** | IR sensors, line followers | $20-50 | Enhanced navigation |
| **Gripper** | Simple parallel jaw | $50-100 | Basic manipulation |
| **Extended Battery** | Larger capacity pack | $30-80 | Longer operation |
| **Custom Chassis** | Different form factor | $50-150 | Specialized applications |

## Computing Platform

### Raspberry Pi 4 Specifications
| Component | Specification | Notes |
|-----------|---------------|-------|
| **CPU** | 4x Cortex-A72 | 1.5 GHz per core |
| **GPU** | VideoCore VI | OpenGL ES 3.0 |
| **Memory** | 8GB LPDDR4 | 32GB/s bandwidth |
| **Storage** | 128GB MicroSD | Class 10, UHS-I |
| **Connectivity** | WiFi 5, Bluetooth 5.0, Ethernet | Multiple interfaces |

### Performance Capabilities
- **ROS 2 Compatibility**: Full ROS 2 Humble support
- **Computer Vision**: OpenCV with hardware acceleration
- **Navigation**: Basic SLAM and path planning
- **Real-time Control**: Adequate for basic robotics tasks

### Power Management
| Component | Power Consumption | Voltage | Notes |
|-----------|------------------|---------|-------|
| **Raspberry Pi 4** | 3-6W | 5V | Variable based on load |
| **Motors** | 10-30W | 11.1V | Peak during acceleration |
| **Cameras** | 1-2W | 3.3V | Continuous operation |
| **Sensors** | 2-5W | Various | Multiple sensors |

## Sensor Integration

### Camera System
| Camera | Purpose | Resolution | FPS | Interface |
|--------|---------|------------|-----|-----------|
| **Pi Camera v3** | Navigation, object detection | 4608x2592 | 50@1080p | CSI |

### Ultrasonic Sensors
| Parameter | Value | Notes |
|-----------|-------|-------|
| **Range** | 2cm - 400cm | Effective range |
| **Accuracy** | ±3mm | At 1m distance |
| **Update Rate** | 10-20 Hz | Configurable |
| **Field of View** | 15° | Cone-shaped detection |

### IMU Capabilities
| Sensor | Range | Resolution | Bandwidth |
|--------|-------|------------|-----------|
| **Accelerometer** | ±16g | 0.488 mg | 1.1 kHz |
| **Gyroscope** | ±2000 dps | 70 mdps | 1.1 kHz |
| **Magnetometer** | ±4800 µT | 0.15 µT | 100 Hz |

## Software Stack

### Pre-installed Software
| Component | Version | Purpose |
|-----------|---------|---------|
| **ROS 2** | Humble Hawksbill | Robot middleware |
| **OpenCV** | 4.5+ | Computer vision |
| **Python** | 3.8+ | Primary development language |
| **Navigation2** | Latest | Path planning |
| **Rviz2** | Latest | Visualization |

### Basic Software Components
- **Robot Drivers**: Low-level hardware interfaces
- **Navigation Stack**: Basic path planning and obstacle avoidance
- **Camera Interface**: Image capture and processing
- **Communication**: WiFi-based remote operation
- **Simulation**: Gazebo integration for testing

## Performance Benchmarks

### Computational Performance
| Task | Performance | Notes |
|------|-------------|-------|
| **Object Detection** | 5-10 FPS | Simple algorithms |
| **SLAM** | Limited | Basic mapping only |
| **Path Planning** | < 500ms | Simple planners |
| **Control Loop** | 50-100 Hz | Adequate for basic tasks |

### Navigation Performance
| Metric | Target | Typical Performance |
|--------|--------|-------------------|
| **Localization Accuracy** | < 10cm | 10-20cm |
| **Navigation Success Rate** | > 70% | 70-85% (simple environments) |
| **Path Efficiency** | < 1.5x optimal | 1.3x optimal |
| **Obstacle Avoidance** | 90% | 85-90% (detectable obstacles) |

## Educational Applications

### Course Integration
- **ROS 2 Fundamentals**: Basic node creation and communication
- **Navigation Basics**: Simple path planning and obstacle avoidance
- **Perception**: Basic image processing and object detection
- **Control Systems**: Simple feedback control loops

### Project Examples
1. **Line Following**: Basic autonomous navigation
2. **Obstacle Avoidance**: Simple reactive navigation
3. **Object Tracking**: Basic computer vision
4. **Mapping**: Simple grid-based mapping
5. **Multi-Robot**: Basic coordination (with multiple kits)

## Cost Analysis

### Base Configuration
| Component | Cost Range | Source | Notes |
|-----------|------------|---------|-------|
| **Raspberry Pi 4 (8GB)** | $100-120 | Raspberry Pi Foundation | Main computer |
| **Chassis Kit** | $80-120 | 3D printing service | Custom design |
| **Motors & Wheels** | $60-80 | Online retailers | 2x gear motors |
| **Motor Controllers** | $30-50 | Online retailers | 2x ESC |
| **Power System** | $40-60 | Battery suppliers | LiPo + charger |
| **Sensors** | $50-80 | Electronics retailers | Camera, ultrasonic, IMU |
| **Electronics** | $30-50 | Local suppliers | Cables, connectors |
| **Total Base Kit** | $390-560 | Various | Without tools |

### Optional Additions
| Addition | Cost | Purpose |
|----------|------|---------|
| **Additional Sensors** | $20-50 | Enhanced perception |
| **Gripper** | $50-100 | Basic manipulation |
| **Extended Battery** | $30-80 | Longer operation |
| **Carrying Case** | $40-60 | Transport and storage |
| **Tools Kit** | $30-50 | Assembly and maintenance |

### Complete Setup (10 units)
| Component | Quantity | Unit Cost | Total Cost |
|-----------|----------|-----------|------------|
| **Economy Robot Kits** | 10 | $500 | $5,000 |
| **Laptops/PCs** | 10 | $500 | $5,000 |
| **Network Equipment** | 1 set | $500 | $500 |
| **Furniture** | 10 stations | $200 | $2,000 |
| **Tools & Consumables** | 1 set | $300 | $300 |
| **Backup Parts** | 1 set | $500 | $500 |
| **Total Lab Setup** | | | **$13,300** |

## Comparison with Alternatives

### vs. Edge Kit
| Aspect | Economy Kit | Edge Kit |
|--------|-------------|----------|
| **Price** | $500 | $5,000 |
| **Performance** | Basic | Advanced |
| **Sensors** | Minimal | Comprehensive |
| **AI Capabilities** | Limited | Advanced |
| **Applications** | Learning/Basic | Research/Advanced |

### vs. Simulation Only
| Aspect | Economy Kit | Simulation Only |
|--------|-------------|-----------------|
| **Cost** | $500 | $0-200 |
| **Real Experience** | Physical interaction | Virtual only |
| **Hardware Skills** | Yes | No |
| **Real-world Challenges** | Yes | No |
| **Maintenance** | Required | None |

## Curriculum Coverage

### Week 1-2: ROS 2 Basics
- ROS 2 installation and setup
- Basic commands and tools
- Node creation and communication
- Parameter servers

### Week 3-4: Robot Control
- Publisher-subscriber patterns
- Basic movement commands
- Sensor data reading
- Simple control loops

### Week 5-6: Navigation
- Basic path planning
- Obstacle avoidance
- Odometry and localization
- Simple SLAM concepts

### Week 7-8: Perception
- Camera image processing
- Basic object detection
- Sensor fusion
- Integration project

## Maintenance and Support

### Expected Lifespan
- **Mechanical Components**: 3-5 years with maintenance
- **Electronics**: 2-3 years (obsolescence factor)
- **Battery**: 1-2 years (cycle life)
- **Software**: Continuous updates available

### Common Issues and Solutions
| Issue | Cause | Solution |
|-------|-------|----------|
| **Motor Stalling** | Insufficient power | Check battery voltage |
| **Camera Not Working** | CSI connection | Reconnect ribbon cable |
| **WiFi Disconnection** | Signal interference | Move away from interference |
| **Sensor Noise** | Electrical interference | Shield cables, add filters |

### Support Resources
- **Documentation**: Online tutorials and guides
- **Community**: Raspberry Pi and ROS communities
- **Training**: Online courses and workshops
- **Warranty**: 1-year component warranty

## Purchase Recommendations

### For Individual Learners
- Start with base configuration
- Add sensors based on interests
- Consider backup batteries for extended use

### For Educational Institutions
- Purchase 10-15 units for lab setup
- Standardize on one configuration
- Budget for spare parts and maintenance
- Consider bulk purchasing discounts

### For Makerspaces
- Shared equipment model
- Rental/usage fees
- Maintenance volunteers
- Graduated complexity options

## Advancement Pathways

### From Economy to Edge
- **Software Skills**: Direct transfer of ROS 2 knowledge
- **Conceptual Understanding**: Foundation for advanced concepts
- **Project Experience**: Portfolio development
- **Hardware Transition**: Familiar architecture concepts

### Recommended Upgrades
1. **Additional Sensors**: Enhance perception capabilities
2. **Better Camera**: Improved computer vision
3. **More Processing**: External compute for complex tasks
4. **Advanced Navigation**: LIDAR integration

## Limitations and Considerations

### Performance Limitations
- **Computation**: Limited by Raspberry Pi capabilities
- **Real-time**: Not suitable for high-speed applications
- **AI**: Basic machine learning only
- **Multi-sensor**: Limited processing of multiple sensors

### Environmental Limitations
- **Indoor Use**: Not weatherproof
- **Smooth Surfaces**: Best on flat, smooth floors
- **Lighting**: Camera performance varies with lighting
- **Space**: Requires moderate space for operation

## Conclusion

The Economy Kit provides an accessible entry point into Physical AI and robotics education. While limited in performance compared to advanced platforms, it offers:

- **Cost-effective**: Low barrier to entry
- **Educational Value**: Core concepts learning
- **Hands-on Experience**: Physical interaction with robots
- **Foundation Building**: Skills for advanced platforms
- **Project Development**: Practical application experience

This platform is ideal for introductory courses, hobby projects, and institutions with limited budgets who still want to provide hands-on robotics education.