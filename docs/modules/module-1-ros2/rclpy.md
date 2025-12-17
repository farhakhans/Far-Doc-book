---
sidebar_position: 3
---

# rclpy: Python Client Library for ROS 2

## Introduction to rclpy

`rclpy` is the Python client library for ROS 2. It provides Python bindings for the ROS 2 client library (rcl), allowing you to write ROS 2 nodes in Python. This makes it accessible for rapid prototyping and development of robotic applications.

## Installation and Setup

`rclpy` is typically installed as part of the ROS 2 distribution. If you're using a standard ROS 2 installation, it should already be available. You can install it separately using:

```bash
pip install rclpy
```

## Basic Node Structure

Every ROS 2 Python node follows a similar structure:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Node initialization code here

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Publishers

Publishers allow nodes to send messages to topics:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        # Create a publisher
        self.publisher = self.create_publisher(
            String,           # Message type
            'topic_name',     # Topic name
            10                # Queue size
        )

        # Create a timer to publish periodically
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = Talker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Subscribers

Subscribers allow nodes to receive messages from topics:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')

        # Create a subscription
        self.subscription = self.create_subscription(
            String,           # Message type
            'topic_name',     # Topic name
            self.listener_callback,  # Callback function
            10                # Queue size
        )
        # Prevent unused variable warning
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = Listener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Services

Services enable request-response communication:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')

        # Create a service
        self.srv = self.create_service(
            AddTwoInts,           # Service type
            'add_two_ints',       # Service name
            self.add_two_ints_callback  # Callback function
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MinimalService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Clients

Clients call services:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')

        # Create a client
        self.cli = self.create_client(
            AddTwoInts,      # Service type
            'add_two_ints'   # Service name
        )

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        self.request.a = a
        self.request.b = b
        self.future = self.cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = MinimalClient()

    response = client.send_request(1, 2)
    client.get_logger().info(f'Result: {response.sum}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with Parameters

Parameters allow runtime configuration of nodes:

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('param_name', 'default_value')
        self.declare_parameter('int_param', 42)
        self.declare_parameter('float_param', 3.14)

        # Get parameter values
        param_value = self.get_parameter('param_name').value
        int_value = self.get_parameter('int_param').value
        float_value = self.get_parameter('float_param').value

        self.get_logger().info(f'Parameter values: {param_value}, {int_value}, {float_value}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Custom Message Types

To use custom message types, create them in a package and import them:

```python
# Assuming you have a custom message RobotPose in your_package_msgs
from your_package_msgs.msg import RobotPose

class CustomMessageNode(Node):
    def __init__(self):
        super().__init__('custom_message_node')

        # Use custom message in publisher
        self.publisher = self.create_publisher(RobotPose, 'robot_pose', 10)

        # Use custom message in subscriber
        self.subscription = self.create_subscription(
            RobotPose,
            'robot_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        self.get_logger().info(f'Robot position: ({msg.x}, {msg.y}, {msg.z})')
```

## Async and Await Support

rclpy supports asyncio for more complex applications:

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import asyncio

class AsyncNode(Node):
    def __init__(self):
        super().__init__('async_node')

        # Create callback groups for async execution
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()

def main(args=None):
    rclpy.init(args=args)
    node = AsyncNode()

    # Use multi-threaded executor for async operations
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Error Handling and Logging

Proper error handling is crucial for robust robotic applications:

```python
import rclpy
from rclpy.node import Node
import traceback

class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')

    def safe_operation(self):
        try:
            # Potentially risky operation
            result = self.perform_operation()
            self.get_logger().info(f'Operation successful: {result}')
        except Exception as e:
            self.get_logger().error(f'Operation failed: {str(e)}')
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')

def main(args=None):
    rclpy.init(args=args)
    node = RobustNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Unhandled exception: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Exercise: Robot Controller

Create a ROS 2 node that:

1. Subscribes to a "cmd_vel" topic to receive velocity commands
2. Publishes to a "robot_state" topic with the robot's current state
3. Uses parameters to configure robot properties (max speed, etc.)
4. Implements a service that returns the robot's current position

This exercise will help you practice using rclpy for common robotic applications.

## Summary

`rclpy` provides a powerful and accessible way to develop ROS 2 nodes in Python:

- Simple node structure with proper initialization and cleanup
- Easy creation of publishers, subscribers, services, and clients
- Parameter handling for runtime configuration
- Support for custom message types
- Error handling and logging capabilities
- Async support for complex applications

Mastering rclpy is essential for developing Python-based robotic applications in ROS 2.