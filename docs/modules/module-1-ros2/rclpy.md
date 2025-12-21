---
sidebar_position: 3
---

# rclpy: Python Client Library for ROS2

rclpy is the Python client library for ROS2, providing Python APIs to interact with ROS2 concepts like nodes, publishers, subscribers, services, and actions. This section covers the essential aspects of using rclpy for robotics development.

## Introduction to rclpy

rclpy serves as a Python wrapper around the ROS Client Library (rcl), providing a Pythonic interface to ROS2 functionality. It allows Python developers to create ROS2 nodes and interact with the ROS2 ecosystem.

### Basic Node Structure

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Initialize publishers, subscribers, services, etc.

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)  # Keep node running
    node.destroy_node()
    rclpy.shutdown()
```

## Publishers and Subscribers

### Creating Publishers

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(
            String,           # Message type
            'topic_name',     # Topic name
            10               # Queue size (QoS history depth)
        )
        self.timer = self.create_timer(0.5, self.publish_message)
        self.counter = 0

    def publish_message(self):
        msg = String()
        msg.data = f'Message {self.counter}'
        self.publisher.publish(msg)
        self.counter += 1
```

### Creating Subscribers

```python
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,           # Message type
            'topic_name',     # Topic name
            self.listener_callback,  # Callback function
            10                # Queue size
        )
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
```

## Services

### Service Server

```python
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.server = self.create_service(
            AddTwoInts,                    # Service type
            'add_two_ints',                # Service name
            self.add_callback              # Callback function
        )

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

### Service Client

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(
            AddTwoInts,           # Service type
            'add_two_ints'        # Service name
        )
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        return future
```

## Actions

Actions provide a way to handle long-running tasks with feedback and status updates.

### Action Server

```python
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )

            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

### Action Client

```python
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(
            f'Received feedback: {feedback_msg.feedback.sequence}'
        )
```

## Parameters

Parameters allow runtime configuration of nodes:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('my_param', 'default_value')
        self.declare_parameter('threshold', 1.0)

        # Get parameter values
        self.param_value = self.get_parameter('my_param').value
        self.threshold = self.get_parameter('threshold').value

        # Set parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'threshold' and param.type_ == Parameter.Type.DOUBLE:
                self.threshold = param.value
        return SetParametersResult(successful=True)
```

## Timers

Timers allow periodic execution of functions:

```python
class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')
        self.timer = self.create_timer(
            0.5,  # Period in seconds
            self.timer_callback
        )
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f'Timer callback {self.counter}')
        self.counter += 1
```

## Quality of Service (QoS)

QoS settings control message delivery characteristics:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Create custom QoS profile
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# Use with publisher/subscriber
publisher = self.create_publisher(String, 'topic', qos_profile)
```

## Error Handling and Logging

### Logging

```python
class LoggingNode(Node):
    def __init__(self):
        super().__init__('logging_node')

    def some_method(self):
        self.get_logger().debug('Debug message')
        self.get_logger().info('Info message')
        self.get_logger().warn('Warning message')
        self.get_logger().error('Error message')
        self.get_logger().fatal('Fatal message')
```

### Exception Handling

```python
def safe_method(self):
    try:
        # ROS2 operations
        pass
    except Exception as e:
        self.get_logger().error(f'Error occurred: {str(e)}')
```

## Best Practices

1. **Node Lifecycle**: Always call `destroy_node()` and `shutdown()`
2. **Resource Management**: Properly clean up publishers, subscribers, and services
3. **Threading**: Use appropriate executor types for concurrent operations
4. **Error Handling**: Implement proper exception handling
5. **Logging**: Use appropriate log levels for different types of messages
6. **Parameter Validation**: Validate parameters before use

## Advanced Topics

### Custom Message Types

Creating custom messages requires defining `.msg` files in the `msg/` directory of a package and generating Python modules.

### Launch Files

Using launch files to start multiple nodes simultaneously with specific configurations.

### Testing

Using `launch_testing` and `pytest` for testing ROS2 nodes.

Understanding rclpy is fundamental to developing Python-based ROS2 applications. The practical exercises will help you apply these concepts to real robotics problems.