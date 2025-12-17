---
sidebar_position: 2
---

# ROS 2 Nodes, Topics, and Services

## Core Communication Patterns

ROS 2 uses a distributed computing architecture where different processes (nodes) communicate with each other through messages. Understanding the fundamental communication patterns is crucial for developing effective robotic systems.

## Nodes: The Basic Computational Units

A **node** is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 system. In Python, nodes are created by subclassing the `Node` class from the `rclpy` library.

### Creating a Node

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Key Node Concepts

- **Node names**: Must be unique within a ROS 2 domain
- **Lifecycle**: Nodes are initialized, perform work, and then shut down
- **Namespaces**: Allow organization of nodes into logical groups
- **Parameters**: Configurable values that can be set at runtime

## Topics: Publish-Subscribe Communication

**Topics** enable asynchronous, many-to-many communication between nodes using a publish-subscribe pattern.

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()
```

### Topic Communication Characteristics

- **Asynchronous**: Publishers and subscribers don't need to be synchronized
- **Many-to-many**: Multiple publishers can send to the same topic; multiple subscribers can listen to the same topic
- **Data-driven**: Communication occurs when data is available
- **Reliable**: Messages are delivered in the order they were sent

## Services: Request-Response Communication

**Services** enable synchronous, one-to-one communication using a request-response pattern.

### Service Server Example

```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()
```

### Service Client Example

```python
import sys
from add_two_ints_client_py.client import add_two_ints
from add_two_ints_client_py.client_async import add_two_ints_async

def main():
    if len(sys.argv) != 3:
        print('Usage: python client.py <int1> <int2>')
        return

    rclpy.init()
    try:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
        result = add_two_ints(a, b)
        print('%d + %d = %d' % (a, b, result))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
```

### Service Communication Characteristics

- **Synchronous**: Client waits for response from server
- **One-to-one**: Single client communicates with single server
- **Request-response**: Client sends request, server sends response
- **Blocking**: Client blocks until response is received (unless using async)

## Quality of Service (QoS) Settings

ROS 2 provides Quality of Service profiles to configure communication behavior:

```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

# Create a QoS profile with specific settings
qos_profile = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE
)

# Use the QoS profile when creating publisher/subscriber
publisher = self.create_publisher(String, 'topic', qos_profile)
```

### Common QoS Settings

- **Reliability**: BEST_EFFORT or RELIABLE
- **Durability**: VOLATILE or TRANSIENT_LOCAL
- **History**: KEEP_LAST or KEEP_ALL
- **Depth**: Size of message queue

## Practical Exercise: Simple Publisher-Subscriber

Create a simple ROS 2 system with a publisher that sends robot position data and a subscriber that processes this data:

1. Create a publisher node that publishes position data (x, y coordinates)
2. Create a subscriber node that receives position data and logs it
3. Use appropriate message types (geometry_msgs/Point or custom message)
4. Test the communication between nodes

This exercise will help you understand the publish-subscribe pattern and practice creating ROS 2 nodes.

## Summary

ROS 2's communication patterns form the foundation of distributed robotic systems:

- **Nodes** are the computational units
- **Topics** enable asynchronous, many-to-many communication
- **Services** enable synchronous, one-to-one communication
- **QoS settings** allow fine-tuning of communication behavior

Understanding these patterns is essential for building complex robotic systems that can effectively coordinate between multiple components.