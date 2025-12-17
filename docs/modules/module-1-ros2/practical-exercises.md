---
sidebar_position: 5
---

# Module 1: Practical Exercises

<div class="practical-exercise">

## Exercise 1: Simple Publisher-Subscriber System

### Objective
Create a ROS 2 system with a publisher that sends temperature data and a subscriber that processes this data.

### Requirements
1. Create a publisher node that publishes temperature readings (std_msgs/Float64)
2. Create a subscriber node that receives temperature data and logs it
3. Add a parameter to configure the publishing rate
4. Use appropriate QoS settings for reliable communication
5. Test the system by running both nodes simultaneously

</div>

### Steps
1. Create a new package: `ros2 pkg create --build-type ament_python temp_monitor`
2. Implement the publisher in `temp_publisher.py`
3. Implement the subscriber in `temp_subscriber.py`
4. Create a launch file to run both nodes
5. Test the communication between nodes

### Solution Template
```python
# temp_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class TempPublisher(Node):
    def __init__(self):
        super().__init__('temp_publisher')
        self.publisher = self.create_publisher(Float64, 'temperature', 10)
        self.declare_parameter('publish_rate', 1.0)
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0/rate, self.publish_temp)
        self.temp = 20.0  # Starting temperature

    def publish_temp(self):
        msg = Float64()
        # Simulate temperature fluctuation
        self.temp += random.uniform(-0.5, 0.5)
        msg.data = self.temp
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing temperature: {msg.data:.2f}°C')

def main(args=None):
    rclpy.init(args=args)
    node = TempPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 2: Service-Based Robot Control

### Objective
Create a service that allows external nodes to command a robot to move to specific positions.

### Requirements
1. Create a service server that accepts position commands (x, y coordinates)
2. Create a service client that sends position commands
3. The service should return whether the robot can reach the requested position
4. Add validation to ensure positions are within valid ranges

### Custom Service Definition
Create a service file `MoveToPosition.srv`:
```
# Request
float64 x
float64 y
---
# Response
bool success
string message
```

### Implementation
1. Define the custom service message
2. Implement the service server
3. Implement the client
4. Test with various position requests

## Exercise 3: Robot Model with URDF

### Objective
Create a URDF model for a simple mobile robot with differential drive.

### Requirements
1. Create a base link with appropriate dimensions
2. Add two wheels as separate links
3. Connect wheels to base with continuous joints
4. Include visual, collision, and inertial properties
5. Use Xacro to make the model parameterizable

### URDF Structure
Your URDF should include:
- Base link (rectangular chassis)
- Left and right wheels
- Joints connecting wheels to base
- Proper materials and colors
- Valid inertial properties

### Validation
1. Use `check_urdf` to validate your model
2. Visualize in RViz2
3. Load in Gazebo simulation

## Exercise 4: Parameter Server Integration

### Objective
Create a ROS 2 node that uses parameters for configuration and demonstrates parameter callbacks.

### Requirements
1. Create a node that declares multiple parameters (int, double, string, bool)
2. Implement a parameter callback to handle parameter changes at runtime
3. Use the parameters to control node behavior
4. Demonstrate changing parameters using `ros2 param` commands

### Example Implementation
```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParameterDemoNode(Node):
    def __init__(self):
        super().__init__('parameter_demo')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('is_active', True)

        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Use parameters in node behavior
        self.timer = self.create_timer(1.0, self.timer_callback)

    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f'Parameter {param.name} changed to {param.value}')
        return SetParametersResult(successful=True)

    def timer_callback(self):
        name = self.get_parameter('robot_name').value
        speed = self.get_parameter('max_speed').value
        active = self.get_parameter('is_active').value

        self.get_logger().info(f'Robot {name} running at {speed}m/s, active: {active}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterDemoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Exercise 5: Integrated System

### Objective
Combine all concepts learned in Module 1 into a single integrated system.

### Requirements
1. Create a robot node that publishes its position (using a timer)
2. Create a controller node that subscribes to position and publishes velocity commands
3. Create a service that allows external nodes to set the robot's target position
4. Use parameters to configure robot properties (max speed, acceleration)
5. Include a URDF model for the robot
6. Create a launch file that starts all nodes

### System Architecture
```
[Target Service] --> [Controller] --> [Robot Model]
                      ^     |
                      |     |
                   [Position Feedback]
```

### Implementation Steps
1. Create the robot model node (publisher)
2. Create the controller node (subscriber + publisher)
3. Create the target service node
4. Create the URDF model
5. Write a launch file to start all nodes
6. Test the integrated system

## Exercise 6: Debugging and Monitoring

### Objective
Learn to use ROS 2 tools for debugging and monitoring your robotic systems.

### Requirements
1. Use `ros2 topic list` and `ros2 topic echo` to monitor topics
2. Use `ros2 service list` and `ros2 service call` to test services
3. Use `ros2 param list` and `ros2 param get/set` to manage parameters
4. Use `rqt_graph` to visualize the node graph
5. Implement logging in your nodes at different levels (info, warn, error)

### Tools to Practice
- `ros2 topic list/echo/hz/type`
- `ros2 service list/call/type`
- `ros2 param list/get/set/describe`
- `ros2 node list/info`
- `rqt_graph`
- `rqt_console`

## Assessment Rubric

Each exercise will be assessed based on:

- **Functionality** (40%): Does the code work as expected?
- **Code Quality** (25%): Is the code well-structured and commented?
- **ROS 2 Concepts** (25%): Are ROS 2 concepts properly applied?
- **Documentation** (10%): Is the code properly documented?

## Submission Guidelines

1. Create a GitHub repository for your exercises
2. Organize code by exercise number
3. Include a README.md with instructions for running each exercise
4. Include screenshots or logs demonstrating successful execution
5. Submit the repository URL for assessment

## Additional Resources

- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [rclpy Documentation](https://docs.ros2.org/latest/api/rclpy/)

These exercises provide hands-on experience with the core concepts of ROS 2, preparing you for more advanced topics in subsequent modules.