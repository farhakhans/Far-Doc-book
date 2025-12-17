---
sidebar_position: 3
---

# Module 2 Assessment: Simulation Environment Project

## Project Overview

The Simulation Environment Project evaluates your ability to create and utilize simulation environments for safe robot development. You will design and implement comprehensive simulation scenarios using both Gazebo and Unity (or Isaac Sim), demonstrating physics simulation, sensor modeling, and safe testing of control algorithms.

## Learning Outcomes Assessed

This project directly evaluates:
- **LO-003**: Design simulation-based testing environments
- **LO-004**: Utilize NVIDIA Isaac Platform tools
- **LO-006**: Integrate and deploy complete Physical AI systems

## Project Requirements

### Core Components
1. **Multi-Environment Simulation**: Implement simulation in at least 2 different environments
2. **Physics Modeling**: Accurate physics simulation with realistic parameters
3. **Sensor Integration**: Multiple sensor types with realistic noise models
4. **Robot Control**: Integration of control algorithms with simulation
5. **Validation System**: Comparison and validation between simulation environments

### Technical Specifications

#### Environment Requirements
- **Gazebo Environment**: Complete indoor/outdoor scene with obstacles
- **Unity/Isaac Sim Environment**: Equivalent scene with enhanced visuals
- **Physics Properties**: Accurate mass, friction, and collision properties
- **Lighting and Materials**: Realistic environmental conditions

#### Robot and Sensor Requirements
- **Robot Model**: URDF model with 4+ degrees of freedom
- **Camera Sensors**: RGB and depth camera with realistic parameters
- **LIDAR Sensor**: 2D or 3D LIDAR with proper noise modeling
- **IMU Simulation**: Accelerometer and gyroscope with bias and noise
- **Other Sensors**: At least 2 additional sensor types (e.g., force/torque, GPS)

#### Control Integration Requirements
- **Navigation System**: Integration with ROS 2 navigation stack
- **Motion Control**: Low-level control algorithms
- **Safety Systems**: Collision avoidance and emergency stops
- **Performance Monitoring**: Real-time performance metrics

## Implementation Guidelines

### Project Structure
```
simulation_project/
├── gazebo_sim/
│   ├── worlds/
│   ├── models/
│   ├── launch/
│   └── config/
├── unity_sim/ (or isaac_sim/)
│   ├── assets/
│   ├── scenes/
│   └── scripts/
├── robot_control/
│   ├── controllers/
│   ├── launch/
│   └── config/
├── validation/
│   └── comparison_tools/
├── CMakeLists.txt
├── package.xml
└── README.md
```

### Gazebo Implementation Example

#### World File Example
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simulation_project_world">
    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Custom environment elements -->
    <model name="wall_1">
      <pose>0 5 1 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Add furniture and obstacles -->
    <model name="table">
      <pose>2 0 0.5 0 0 0</pose>
      <include>
        <uri>model://table</uri>
      </include>
    </model>

    <!-- Physics configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

#### Sensor Integration Example
```xml
<model name="robot_with_sensors">
  <include>
    <uri>model://my_robot</uri>
  </include>

  <sensor name="camera" type="camera">
    <pose>0.2 0 0.1 0 0 0</pose>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>

  <sensor name="lidar" type="ray">
    <pose>0.1 0 0.2 0 0 0</pose>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</model>
```

### Control System Integration
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

class SimulationController(Node):
    def __init__(self):
        super().__init__('simulation_controller')

        # Subscriptions for all sensor types
        self.scan_subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.image_subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.imu_subscription = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)
        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        # Publisher for robot commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Robot state
        self.safety_distance = 0.5
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # Performance monitoring
        self.start_time = self.get_clock().now()
        self.movement_time = 0.0

    def scan_callback(self, msg):
        """Process LIDAR data for obstacle avoidance"""
        # Find minimum distance
        valid_ranges = [r for r in msg.ranges if not (r != r or r > 10.0)]  # Remove NaN and inf
        if valid_ranges:
            min_distance = min(valid_ranges)

            # Simple navigation algorithm
            cmd_msg = Twist()
            if min_distance < self.safety_distance:
                # Turn to avoid obstacle
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.5
            else:
                # Move forward with slight random turn to explore
                cmd_msg.linear.x = 0.3
                cmd_msg.angular.z = np.random.uniform(-0.1, 0.1)

            self.cmd_publisher.publish(cmd_msg)

    def image_callback(self, msg):
        """Process camera data for visual navigation"""
        # In a real implementation, process image data
        # For this example, just log the image dimensions
        width = msg.width
        height = msg.height
        self.get_logger().info(f'Image received: {width}x{height}')

    def imu_callback(self, msg):
        """Process IMU data for orientation and acceleration"""
        # Extract orientation and angular velocity
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration

        # Use IMU data for improved navigation
        pass

    def odom_callback(self, msg):
        """Process odometry data for localization"""
        # Extract position and orientation from odometry
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Update movement time for performance metrics
        current_time = self.get_clock().now()
        self.movement_time = (current_time - self.start_time).nanoseconds / 1e9

def main(args=None):
    rclpy.init(args=args)
    controller = SimulationController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Assessment Criteria

### Simulation Quality (40%)
- **Physics Accuracy**: Realistic physics simulation with proper parameters (15%)
- **Sensor Modeling**: Accurate sensor simulation with realistic noise (15%)
- **Environment Design**: Well-designed and challenging environments (10%)

### Integration and Functionality (35%)
- **Multi-Environment**: Successful implementation across different simulators (15%)
- **Control Integration**: Effective robot control in simulation (10%)
- **System Performance**: Efficient and stable simulation operation (10%)

### Validation and Analysis (25%)
- **Cross-Validation**: Comparison between different simulation environments (10%)
- **Performance Metrics**: Comprehensive performance analysis (10%)
- **Documentation**: Clear analysis of findings and differences (5%)

## Submission Requirements

1. **Simulation Environments**: Complete Gazebo and Unity/Isaac Sim implementations
2. **Robot Model**: URDF model with sensor integration
3. **Control System**: ROS 2 nodes for robot control
4. **Launch Files**: Complete launch files for each environment
5. **Validation Tools**: Scripts for comparing simulation results
6. **Documentation**: Comprehensive documentation and analysis
7. **Demonstration**: Video showing robot operation in both environments

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
      <td>Physics Simulation</td>
      <td>Highly realistic with advanced features</td>
      <td>Good physics with minor issues</td>
      <td>Basic physics working</td>
      <td>Poor physics implementation</td>
    </tr>
    <tr>
      <td>Sensor Modeling</td>
      <td>Accurate sensors with realistic noise models</td>
      <td>Good sensor modeling</td>
      <td>Basic sensor simulation</td>
      <td>Poor sensor implementation</td>
    </tr>
    <tr>
      <td>Multi-Environment</td>
      <td>Seamless operation across all environments</td>
      <td>Good cross-environment operation</td>
      <td>Basic multi-environment setup</td>
      <td>Limited environment support</td>
    </tr>
    <tr>
      <td>Control Integration</td>
      <td>Sophisticated control algorithms</td>
      <td>Good control integration</td>
      <td>Basic control working</td>
      <td>Poor control integration</td>
    </tr>
    <tr>
      <td>Validation</td>
      <td>Comprehensive validation with insights</td>
      <td>Good validation analysis</td>
      <td>Basic validation</td>
      <td>Limited validation</td>
    </tr>
  </tbody>
</table>

## Performance Metrics to Track

### Simulation Performance
- **Frame Rate**: Maintain 30+ FPS for real-time operation
- **Physics Update Rate**: Consistent physics updates
- **Sensor Update Rates**: Proper timing for different sensors

### Robot Performance
- **Navigation Success Rate**: Percentage of successful navigation tasks
- **Obstacle Avoidance**: Effectiveness of collision avoidance
- **Path Efficiency**: Comparison to optimal paths
- **Execution Time**: Time to complete navigation tasks

### Cross-Environment Comparison
- **Behavior Similarity**: Consistency of robot behavior across environments
- **Performance Differences**: Analysis of performance variations
- **Physics Accuracy**: Comparison of physics simulation quality

## Submission Guidelines

- Submit as a Git repository with all simulation assets
- Include detailed setup instructions for each environment
- Provide video demonstrations of robot operation
- Include analysis of differences between simulation environments
- Document performance metrics and validation results

## Resources and References

- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [ROS 2 Navigation](https://navigation.ros.org/)

## Support and Questions

For questions about this project, please:
- Review the simulation module materials
- Attend simulation-focused office hours
- Use the course forum for technical discussions
- Consult with the instructor for complex implementation issues

Remember to validate your simulation results and compare between different environments to understand the strengths and limitations of each platform.