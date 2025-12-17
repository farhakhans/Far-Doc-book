---
sidebar_position: 2
---

# Module 1 Assessment: ROS 2 Fundamentals Project

## Project Overview

The ROS 2 Fundamentals Project assesses your understanding of core ROS 2 concepts including nodes, topics, services, rclpy, and URDF. You will implement a complete robotic system that demonstrates these concepts in an integrated manner.

## Learning Outcomes Assessed

This project directly evaluates:
- **LO-002**: Implement ROS 2 communication patterns
- **LO-003**: Design simulation-based testing environments
- **LO-006**: Integrate and deploy complete Physical AI systems

## Project Requirements

### Core Components
1. **Custom Message Types**: Create and use custom ROS 2 message types
2. **Node Implementation**: Implement at least 4 different types of nodes
3. **Communication Patterns**: Demonstrate publisher-subscriber and service-client patterns
4. **URDF Model**: Create a functional robot model with proper kinematics
5. **Parameter Management**: Use ROS 2 parameters for configuration

### Technical Specifications

#### Node Requirements
- **Sensor Node**: Publishes simulated sensor data (minimum 2 different sensor types)
- **Controller Node**: Subscribes to sensor data and publishes commands
- **Service Node**: Provides robot configuration or control services
- **Parameter Node**: Manages robot configuration parameters

#### Communication Requirements
- Implement at least 3 different topic-based communication patterns
- Create and use custom message types for robot-specific data
- Implement at least 2 different service types for robot control
- Demonstrate proper Quality of Service (QoS) configuration

#### URDF Requirements
- Create a robot model with at least 4 links and 3 joints
- Include proper visual, collision, and inertial properties
- Add appropriate materials and colors
- Use Xacro for parameterized model creation

## Implementation Guidelines

### Project Structure
```
ros2_project/
├── src/
│   ├── robot_bringup/
│   │   ├── launch/
│   │   └── config/
│   ├── robot_description/
│   │   └── urdf/
│   ├── sensor_nodes/
│   ├── controller_nodes/
│   └── service_nodes/
├── CMakeLists.txt
├── package.xml
└── README.md
```

### Node Implementation Examples

#### Sensor Node Example
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import random

class RobotSensorNode(Node):
    def __init__(self):
        super().__init__('robot_sensor_node')

        # Create publishers for different sensor types
        self.distance_publisher = self.create_publisher(Float64, 'distance_sensor', 10)
        self.scan_publisher = self.create_publisher(LaserScan, 'laser_scan', 10)

        # Timer for sensor data publication
        self.timer = self.create_timer(0.1, self.publish_sensor_data)

        # Robot state
        self.distance_reading = 0.0

    def publish_sensor_data(self):
        # Simulate distance sensor
        distance_msg = Float64()
        distance_msg.data = self.distance_reading + random.uniform(-0.01, 0.01)
        self.distance_publisher.publish(distance_msg)

        # Simulate laser scan
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'
        scan_msg.angle_min = -1.57
        scan_msg.angle_max = 1.57
        scan_msg.angle_increment = 0.01
        scan_msg.ranges = [2.0 + random.uniform(-0.1, 0.1) for _ in range(315)]
        self.scan_publisher.publish(scan_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Controller Node Example
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')

        # Subscriptions
        self.scan_subscription = self.create_subscription(
            LaserScan, 'laser_scan', self.scan_callback, 10)

        # Publisher for robot commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Robot state
        self.safe_distance = 0.5
        self.linear_vel = 0.0
        self.angular_vel = 0.0

    def scan_callback(self, msg):
        # Simple obstacle avoidance algorithm
        min_distance = min(msg.ranges)

        cmd_msg = Twist()
        if min_distance < self.safe_distance:
            # Turn away from obstacle
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.5
        else:
            # Move forward
            cmd_msg.linear.x = 0.3
            cmd_msg.angular.z = 0.0

        self.cmd_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Assessment Criteria

### Technical Implementation (60%)
- **Correct ROS 2 patterns**: Proper implementation of communication patterns (20%)
- **URDF quality**: Well-structured robot model with proper properties (15%)
- **Code quality**: Clean, well-documented, and maintainable code (15%)
- **Parameter usage**: Effective use of ROS 2 parameters (10%)

### Functionality (25%)
- **System integration**: All components work together seamlessly (15%)
- **Performance**: Efficient execution and resource usage (10%)

### Documentation (15%)
- **Code documentation**: Proper comments and docstrings (5%)
- **Package documentation**: README with setup and usage instructions (5%)
- **Design rationale**: Explanation of architectural decisions (5%)

## Submission Requirements

1. **Source Code**: Complete, well-organized source code
2. **Launch Files**: Proper launch files to start the complete system
3. **URDF Files**: Complete robot description in URDF/Xacro
4. **Documentation**: README with setup instructions and design explanation
5. **Demonstration**: Video or detailed description of system operation
6. **Testing**: Evidence of testing and validation

## Evaluation Rubric

<table>
  <thead>
    <tr>
      <th>Aspect</th>
      <th>Excellent (90-100%)</th>
      <th>Good (80-89%)</th>
      <th>Adequate (70-79%)</th>
      <th>Needs Improvement (&lt;70%)</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>ROS 2 Implementation</td>
      <td>All patterns implemented perfectly with advanced features</td>
      <td>All required patterns implemented correctly</td>
      <td>Most patterns implemented with minor issues</td>
      <td>Missing or incorrectly implemented patterns</td>
    </tr>
    <tr>
      <td>URDF Quality</td>
      <td>Excellent model with advanced features and optimization</td>
      <td>Good model with proper properties</td>
      <td>Basic model with some issues</td>
      <td>Poor or incomplete model</td>
    </tr>
    <tr>
      <td>System Integration</td>
      <td>Flawless integration with advanced features</td>
      <td>Good integration with minor issues</td>
      <td>Basic integration working</td>
      <td>Poor integration</td>
    </tr>
    <tr>
      <td>Code Quality</td>
      <td>Excellent documentation, structure, and practices</td>
      <td>Good quality with minor issues</td>
      <td>Adequate quality</td>
      <td>Poor quality</td>
    </tr>
    <tr>
      <td>Documentation</td>
      <td>Comprehensive and clear</td>
      <td>Good documentation</td>
      <td>Basic documentation</td>
      <td>Inadequate documentation</td>
    </tr>
  </tbody>
</table>

## Submission Guidelines

- Submit as a Git repository with clear commit history
- Include all necessary files for system operation
- Provide detailed README with setup and usage instructions
- Include a brief video demonstration (or detailed written description)
- Submit through the course management system by the deadline

## Resources and References

- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [rclpy Documentation](https://docs.ros2.org/latest/api/rclpy/)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [ROS 2 Launch Files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)

## Support and Questions

For questions about this project, please:
- Review the course materials and documentation
- Attend office hours or discussion sessions
- Post questions in the course forum
- Contact the instructor directly for clarification

Remember to start early and iterate on your implementation based on testing and feedback.