---
sidebar_position: 4
---

# Navigation2 (Nav2): Autonomous Navigation System

Navigation2 (Nav2) is the state-of-the-art navigation stack for ROS2, providing a complete framework for autonomous robot navigation. It builds upon the lessons learned from ROS1's Navigation stack with improved architecture, performance, and capabilities.

## Introduction to Navigation2

Nav2 provides a complete navigation solution with:
- **Behavior Trees**: Flexible and configurable navigation behavior
- **Plugin Architecture**: Extensible components for different algorithms
- **Recovery Behaviors**: Robust handling of navigation failures
- **Simulation Integration**: Seamless simulation-to-reality transition
- **Visualization Tools**: Comprehensive debugging and monitoring

### Key Features
- **Behavior Tree Execution**: Configurable navigation workflows
- **Costmap Integration**: 2D and 3D obstacle representation
- **Path Planning**: Global and local path planning algorithms
- **Controller Integration**: Robot motion control
- **Recovery Systems**: Automatic failure handling

## Navigation2 Architecture

### System Components
Nav2 consists of several key components working together:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Global       │    │   Local         │    │   Controller    │
│   Planner      │◄──►│   Planner       │◄──►│   Server        │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         ▲                       ▲                       ▲
         │                       │                       │
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Costmap 2D    │    │   Costmap 2D    │    │   Robot         │
│   (Static)      │    │   (Local)       │    │   Interface     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Core Services
- **Navigation Server**: Main service orchestrating navigation
- **Lifecycle Manager**: Component lifecycle management
- **Map Server**: Static map loading and serving
- **AMCL**: Adaptive Monte Carlo Localization
- **BT Navigator**: Behavior tree execution for navigation

## Installation and Setup

### System Requirements
- **ROS2**: Humble Hawksbill or newer
- **OS**: Ubuntu 22.04 LTS
- **Python**: 3.8 or newer
- **C++**: C++17 compiler

### Installation
```bash
# Install Navigation2 packages
sudo apt update
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-nav2-gui

# Install additional dependencies
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-slam-toolbox
```

### Quick Start
```bash
# Launch Nav2 with default configuration
ros2 launch nav2_bringup navigation_launch.py

# Launch with simulation
ros2 launch nav2_bringup tb3_simulation_launch.py
```

## Behavior Trees in Navigation2

### Behavior Tree Concepts
Behavior trees provide a flexible way to define navigation logic:
- **Sequence Nodes**: Execute children in sequence until one fails
- **Fallback Nodes**: Try children until one succeeds
- **Decorator Nodes**: Modify child behavior
- **Action Nodes**: Execute specific navigation tasks

### Example Behavior Tree
```xml
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence name="NavigateWithRecovery">
            <RecoveryNode number_of_retries="2" name="Spin">
                <Spin spin_dist="1.571"/>
                <ClearEntirelyCostmap name="ClearSpin"/>
            </RecoveryNode>
            <RecoveryNode number_of_retries="2" name="Backup">
                <BackUp backup_dist="0.15" backup_speed="0.025"/>
                <ClearEntirelyCostmap name="ClearBackup"/>
            </RecoveryNode>
            <PipelineSequence name="Navigate">
                <RateController hz="10">
                    <RecoveryNode number_of_retries="4" name="PlanToPose">
                        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
                        <SmoothPath path="{path}" smoother_id="simple_smoother"/>
                    </RecoveryNode>
                </RateController>
                <RecoveryNode number_of_retries="4" name="FollowPath">
                    <FollowPath path="{path}" controller_id="FollowPath"/>
                    <ClearEntirelyCostmap name="ClearOscillation"/>
                </RecoveryNode>
            </PipelineSequence>
        </Sequence>
    </BehaviorTree>
</root>
```

### Custom Behavior Trees
Create custom behavior trees for specific navigation requirements:
- Define new action nodes
- Create custom recovery behaviors
- Modify existing navigation logic
- Add domain-specific decision making

## Costmap Configuration

### Costmap Overview
Costmaps represent the environment with different cost values:
- **FREE_SPACE**: Navigable areas (cost: 0)
- **LETHAL_OBSTACLE**: Impassable obstacles (cost: 254)
- **INSCRIBED_INFLATED_OBSTACLE**: Robot-inflated obstacles (cost: 253)
- **UNKNOWN**: Unknown areas (cost: 255)

### Costmap Parameters
```yaml
# Global costmap
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: true
      rolling_window: false
      width: 100
      height: 100
      resolution: 0.05
      origin_x: -25.0
      origin_y: -25.0

      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

# Local costmap
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: true
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05

      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

## Path Planning Algorithms

### Global Planners
Nav2 supports multiple global planning algorithms:

#### NavFn (Dijkstra/A* based)
- Fast grid-based path planning
- Good for simple environments
- Deterministic results

#### Global Planner (A* based)
- Advanced A* implementation
- Better heuristic functions
- More efficient than NavFn

#### CARMA (Curvature and RRT-based)
- Curvature-constrained planning
- Suitable for car-like robots
- Smooth path generation

### Local Planners
Local planners handle dynamic obstacle avoidance:

#### DWA (Dynamic Window Approach)
- Real-time obstacle avoidance
- Dynamic constraint handling
- Velocity-based planning

#### Trajectory Rollout
- Predictive path evaluation
- Robot dynamics consideration
- Multiple trajectory evaluation

#### MPC (Model Predictive Control)
- Advanced control theory
- Predictive optimization
- Smooth trajectory generation

### Custom Planner Development
```python
from nav2_core.global_planner import GlobalPlanner
from nav2_core.types import Costmap
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class CustomPlanner(GlobalPlanner):
    def __init__(self):
        super().__init__()
        self.logger = None

    def configure(self, plugin_name, node, tf, costmap):
        self.logger = node.get_logger()
        self.costmap = costmap
        self.plugin_name = plugin_name
        self.node = node

    def cleanup(self):
        pass

    def set_costmap(self, costmap):
        self.costmap = costmap

    def create_plan(self, start, goal):
        path = Path()
        path.header.frame_id = "map"

        # Implement custom planning algorithm
        current = start
        while not self.is_goal_reached(current, goal):
            next_pose = self.get_next_pose(current, goal)
            path.poses.append(next_pose)
            current = next_pose

            if self.is_path_blocked(current):
                return Path()  # Return empty path if blocked

        return path

    def is_goal_reached(self, current, goal):
        # Check if goal is reached
        distance = self.calculate_distance(current, goal)
        return distance < 0.5  # 0.5m threshold

    def get_next_pose(self, current, goal):
        # Calculate next pose in path
        pose = PoseStamped()
        # Implementation details...
        return pose
```

## Controller Configuration

### Controller Server
The controller server manages robot motion execution:

```yaml
controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: true

    # Controller parameters
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.5
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.8
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.6
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: true
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.25
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785
      max_angular_accel: 3.2
      max_linear_accel: 2.5
```

### Controller Types
- **Pure Pursuit**: Simple path following
- **MPC Controller**: Advanced model predictive control
- **PID Controller**: Traditional PID-based control
- **Regulated Controllers**: Velocity-regulated approaches

## Recovery Behaviors

### Recovery System
Recovery behaviors handle navigation failures gracefully:

#### Spin Recovery
- Rotate in place to clear local minima
- Useful for getting unstuck
- Configurable rotation angle

#### Back Up Recovery
- Move robot backward to clear obstacles
- Safe reverse motion
- Configurable distance and speed

#### Wait Recovery
- Pause navigation for a specified time
- Allow dynamic obstacles to move
- Configurable wait duration

### Custom Recovery Behaviors
```python
from nav2_core.recovery import Recovery
from geometry_msgs.msg import Twist
import rclpy

class CustomRecovery(Recovery):
    def __init__(self):
        super().__init__()

    def configure(self, name, node, tf):
        self.name = name
        self.node = node
        self.tf = tf
        self.cmd_vel_pub = node.create_publisher(Twist, 'cmd_vel', 1)

    def cleanup(self):
        self.node.destroy_publisher(self.cmd_vel_pub)

    def execute(self, initial_pose, goal_pose):
        # Implement custom recovery behavior
        twist = Twist()
        twist.linear.x = 0.1  # Move forward slowly
        twist.angular.z = 0.5  # Rotate while moving

        # Execute recovery for 5 seconds
        start_time = self.node.get_clock().now()
        while rclpy.ok():
            current_time = self.node.get_clock().now()
            if (current_time - start_time).nanoseconds > 5e9:  # 5 seconds
                break
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Stop robot
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)

        return True  # Recovery successful
```

## Simulation and Testing

### Gazebo Integration
Nav2 integrates seamlessly with Gazebo simulation:

```bash
# Launch navigation in simulation
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=false
```

### Testing Scenarios
- **Static Map Navigation**: Navigate to goals in known environments
- **Dynamic Obstacles**: Handle moving obstacles
- **Recovery Testing**: Trigger and test recovery behaviors
- **Multi-goal Navigation**: Navigate through multiple waypoints

### Performance Evaluation
```bash
# Launch navigation performance test
ros2 launch nav2_bringup navigation_performance_test.launch.py
```

## Best Practices

### Configuration Management
1. **Parameter Tuning**: Adjust parameters for specific robots and environments
2. **Layered Configuration**: Use multiple parameter files
3. **Version Control**: Track configuration changes
4. **Documentation**: Document parameter meanings

### Performance Optimization
1. **Costmap Resolution**: Balance accuracy and performance
2. **Update Frequencies**: Match to robot capabilities
3. **Lookahead Distances**: Configure for robot speed
4. **Recovery Behaviors**: Set appropriate retry counts

### Safety Considerations
1. **Velocity Limits**: Set appropriate speed limits
2. **Safety Distances**: Configure minimum obstacle distances
3. **Emergency Stops**: Implement safety stop mechanisms
4. **Monitoring**: Continuously monitor navigation state

## Troubleshooting Common Issues

### Navigation Failures
- Check costmap inflation parameters
- Verify sensor data quality
- Validate robot footprint
- Review path planner configuration

### Performance Issues
- Monitor CPU and memory usage
- Check costmap update frequencies
- Verify sensor data rates
- Optimize behavior tree complexity

### Localization Problems
- Validate AMCL configuration
- Check initial pose accuracy
- Verify map quality
- Monitor TF tree integrity

## Advanced Topics

### Multi-Robot Navigation
- Coordinate multiple robots in shared spaces
- Collision avoidance between robots
- Task allocation and coordination
- Communication protocols

### 3D Navigation
- Volumetric costmaps for 3D navigation
- Multi-story building navigation
- Aerial robot navigation
- Complex terrain navigation

### Learning-Based Navigation
- Reinforcement learning for navigation
- Imitation learning from demonstrations
- Neural network-based path planning
- Adaptive behavior learning

Navigation2 provides a robust and flexible framework for autonomous robot navigation, with extensive configuration options and extensibility for custom applications.