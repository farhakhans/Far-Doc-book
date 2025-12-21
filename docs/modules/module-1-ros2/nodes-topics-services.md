---
sidebar_position: 2
---

# Nodes, Topics, and Services

Understanding the fundamental communication patterns in ROS2 is crucial for building distributed robotic systems. This section covers the three primary communication mechanisms: nodes, topics, and services.

## Nodes

Nodes are the fundamental building blocks of ROS2 applications. Each node is a process that performs computation and communicates with other nodes.

### Node Structure

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Node initialization code

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Node Responsibilities
- Publishing and subscribing to topics
- Providing and calling services
- Creating and managing timers
- Handling parameters
- Logging messages

## Topics and Publishers/Subscribers

Topics enable asynchronous, many-to-many communication through a publish-subscribe pattern.

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

### Quality of Service (QoS) Settings
- Reliability: Best effort vs. reliable delivery
- Durability: Volatile vs. transient local
- History: Keep last vs. keep all
- Depth: Size of message queue

## Services

Services provide synchronous request-response communication between nodes.

### Service Server Example

```python
from example_interfaces.srv import AddTwoInts
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
```

### Service Client Example

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Best Practices

### Node Design
- Keep nodes focused on a single responsibility
- Use meaningful node and topic names
- Implement proper error handling and logging
- Consider resource usage and efficiency

### Topic Design
- Use descriptive topic names following conventions
- Consider message frequency and data size
- Apply appropriate QoS policies
- Document message content and purpose

### Service Design
- Use services for synchronous operations
- Consider timeout and retry mechanisms
- Validate request parameters
- Handle errors gracefully

## Common Patterns

### Publisher-Subscriber Pattern
- Sensors publishing data
- Multiple nodes consuming sensor data
- Decoupled architecture

### Client-Server Pattern
- Action planning services
- Configuration services
- Synchronous computation requests

### Parameter Server Integration
- Dynamic configuration
- Runtime parameter adjustment
- Node-specific settings

Understanding these fundamental communication patterns is essential for building robust and maintainable ROS2 applications. Practice implementing these patterns in the practical exercises to solidify your understanding.