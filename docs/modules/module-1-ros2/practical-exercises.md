---
sidebar_position: 5
---

# Module 1: Practical Exercises

This section contains hands-on exercises to reinforce the concepts learned in the ROS2 Fundamentals module. Complete these exercises to demonstrate your understanding of ROS2 concepts.

## Exercise 1: Basic Publisher and Subscriber

### Objective
Create a simple publisher that sends messages and a subscriber that receives and logs them.

### Requirements
1. Create a publisher node that publishes "Hello, World!" messages every second
2. Create a subscriber node that listens to the same topic and logs received messages
3. Use appropriate message types (String from std_msgs)
4. Implement proper node lifecycle management

### Implementation Steps
1. Create a new ROS2 package: `ros2 pkg create --build-type ament_python publisher_subscriber_ex`
2. Implement the publisher node in `publisher_subscriber_ex/publisher_subscriber_ex/publisher.py`
3. Implement the subscriber node in `publisher_subscriber_ex/publisher_subscriber_ex/subscriber.py`
4. Update setup.py to include entry points
5. Test the nodes with `ros2 run publisher_subscriber_ex publisher` and `ros2 run publisher_subscriber_ex subscriber`

### Verification
- Messages should be published and received correctly
- No errors in the terminal output
- Proper node shutdown

## Exercise 2: Custom Message Service

### Objective
Create a custom service that performs a calculation based on request parameters.

### Requirements
1. Define a custom service message in `.srv` format
2. Implement a service server that performs a mathematical operation
3. Implement a service client that calls the service
4. Add error handling for invalid inputs

### Implementation Steps
1. Create a new ROS2 package: `ros2 pkg create --build-type ament_python custom_service_ex`
2. Create `srv/CalculateArea.srv` with request fields (length, width) and response (area)
3. Implement the service server
4. Implement the service client
5. Test with various inputs including edge cases

### Verification
- Service should respond correctly to valid requests
- Proper error handling for invalid inputs
- Service should be discoverable with `ros2 service list`

## Exercise 3: Robot State Publisher with URDF

### Objective
Create a URDF model for a simple robot and publish its state using robot_state_publisher.

### Requirements
1. Create a URDF file for a simple 2-wheeled robot
2. Include proper visual, collision, and inertial properties
3. Use robot_state_publisher to publish transforms
4. Visualize the robot in RViz

### Implementation Steps
1. Create URDF file with base link and two wheel links
2. Define joints connecting wheels to base
3. Add proper mass and inertial properties
4. Create launch file to start robot_state_publisher
5. Configure RViz to display the robot

### Verification
- Robot displays correctly in RViz
- Joint states are properly published
- TF tree shows correct relationships

## Exercise 4: Parameter Server Integration

### Objective
Create a node that uses ROS2 parameters for runtime configuration.

### Requirements
1. Create a node that declares parameters with default values
2. Use parameters to control node behavior
3. Implement parameter callback for dynamic updates
4. Test parameter changes at runtime

### Implementation Steps
1. Create a new ROS2 package: `ros2 pkg create --build-type ament_python parameter_ex`
2. Implement a node that declares and uses parameters
3. Add parameter callback for validation
4. Test with `ros2 param` commands
5. Create launch file that sets parameters

### Verification
- Parameters can be set at launch time
- Parameters can be changed at runtime
- Parameter validation works correctly
- Node behavior changes based on parameter values

## Exercise 5: Timer-Based Control Node

### Objective
Create a node that uses timers for periodic control tasks.

### Requirements
1. Create a node with multiple timers at different frequencies
2. Implement different behaviors for each timer callback
3. Use parameters to control timer frequencies
4. Implement proper cleanup when node shuts down

### Implementation Steps
1. Create a new ROS2 package: `ros2 pkg create --build-type ament_python timer_control_ex`
2. Implement node with at least 2 timers at different frequencies
3. Use different callbacks for each timer
4. Add parameters to control timer periods
5. Test with various parameter values

### Verification
- Timers execute at expected frequencies
- Different callbacks execute correctly
- Parameters properly control timer behavior
- No resource leaks on shutdown

## Exercise 6: Integrated System

### Objective
Combine all concepts into a single integrated system.

### Requirements
1. Create a robot system with multiple nodes
2. Use topics, services, parameters, and timers
3. Include proper error handling
4. Create launch file to start the entire system

### Implementation Steps
1. Create a main package for the integrated system
2. Implement nodes for sensor simulation, control, and visualization
3. Use services for system commands
4. Use parameters for configuration
5. Create launch file with all required nodes
6. Test complete system integration

### Verification
- All nodes communicate correctly
- System responds to service calls
- Parameters control system behavior
- System starts and stops cleanly

## Assessment Criteria

### Technical Implementation (70%)
- Correct implementation of ROS2 concepts
- Proper error handling and resource management
- Clean, well-structured code
- Appropriate use of ROS2 tools and conventions

### Functionality (20%)
- All required features work correctly
- Proper interaction between components
- Correct handling of edge cases

### Documentation (10%)
- Clear code comments
- Proper README with usage instructions
- Well-structured package organization

## Submission Requirements

For each exercise:
1. Complete source code in proper ROS2 package structure
2. README file with build and run instructions
3. Brief report explaining implementation decisions
4. Screenshots or logs demonstrating functionality

## Additional Resources

- ROS2 tutorials: https://docs.ros.org/en/humble/Tutorials.html
- rclpy documentation: https://docs.ros2.org/latest/api/rclpy/
- URDF tutorials: http://wiki.ros.org/urdf/Tutorials
- Quality of Service guide: https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html

Complete these exercises to demonstrate mastery of ROS2 fundamentals before advancing to Module 2.