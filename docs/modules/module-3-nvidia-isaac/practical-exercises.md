---
sidebar_position: 5
---

# Module 3: Practical Exercises

## Exercise 1: Isaac Sim Environment Setup

### Objective
Set up Isaac Sim with a mobile robot and configure basic sensors for navigation tasks.

### Requirements
1. Install Isaac Sim (using Docker or native installation)
2. Create a simple mobile robot model in Isaac Sim
3. Configure camera and LIDAR sensors
4. Test basic movement and sensor data collection

### Steps
1. **Install Isaac Sim** using Docker:
   ```bash
   docker pull nvcr.io/nvidia/isaac-sim:2023.1.0-hotfix1
   ```

2. **Create a Python script** to set up the environment:
   ```python
   import omni
   from omni.isaac.core import World
   from omni.isaac.core.utils.stage import add_reference_to_stage
   from omni.isaac.core.utils.nucleus import get_assets_root_path
   from omni.isaac.sensor import Camera
   from omni.isaac.range_sensor import LidarRtx
   import numpy as np

   # Initialize Isaac Sim world
   world = World(stage_units_in_meters=1.0)

   # Add ground plane
   world.scene.add_default_ground_plane()

   # Add a simple robot (using NVIDIA's asset library)
   assets_root_path = get_assets_root_path()
   if assets_root_path:
       robot_path = assets_root_path + "/Isaac/Robots/Carter/carter_novisu.usd"
       add_reference_to_stage(usd_path=robot_path, prim_path="/World/Robot")
   ```

3. **Configure sensors** on the robot:
   ```python
   # Add camera sensor
   camera = Camera(
       prim_path="/World/Robot/base_link/Camera",
       frequency=30,
       resolution=(640, 480)
   )

   # Add LIDAR sensor
   lidar = LidarRtx(
       prim_path="/World/Robot/base_link/Lidar",
       translation=np.array([0.1, 0.0, 0.2]),
       config="Example_Rotary_Mechanical_Lidar",
       rotation_frequency=10,
       samples_per_scan=1000,
       update_frequency=10
   )
   ```

4. **Run the simulation** and collect sensor data:
   ```python
   world.reset()
   for i in range(100):
       world.step(render=True)

       if i % 10 == 0:  # Every 10 steps
           # Get camera image
           rgb_image = camera.get_rgb()
           print(f"Captured image at step {i}, shape: {rgb_image.shape}")

           # Get LIDAR scan
           lidar_data = lidar.get_linear_depth_data()
           print(f"LIDAR data shape: {lidar_data.shape}")
   ```

### Expected Outcome
Robot should be spawned in Isaac Sim with functioning camera and LIDAR sensors publishing data.

### Solution Template
```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.robots import Robot
from omni.isaac.sensor import Camera
from omni.isaac.range_sensor import LidarRtx
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import PhysxSchema, UsdPhysics
import numpy as np

class IsaacSimExercise1:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        self.camera = None
        self.lidar = None

    def setup_environment(self):
        """Set up the Isaac Sim environment"""
        # Add ground plane
        self.world.scene.add_default_ground_plane()

        # Add obstacles
        from omni.isaac.core.objects import DynamicCuboid
        for i in range(3):
            self.world.scene.add(
                DynamicCuboid(
                    prim_path=f"/World/obstacle_{i}",
                    name=f"obstacle_{i}",
                    position=[2.0 + i*0.5, 0.0, 0.25],
                    size=0.5,
                    color=[0.5, 0.5, 0.5]
                )
            )

    def add_robot(self):
        """Add a robot to the environment"""
        assets_root_path = get_assets_root_path()
        if assets_root_path:
            robot_path = assets_root_path + "/Isaac/Robots/Carter/carter_novisu.usd"
            add_reference_to_stage(usd_path=robot_path, prim_path="/World/Robot")
        else:
            print("Could not use Isaac Sim assets. Please enable Isaac Sim preview extensions")

    def add_sensors(self):
        """Add sensors to the robot"""
        # Camera
        self.camera = Camera(
            prim_path="/World/Robot/base_link/Camera",
            frequency=30,
            resolution=(640, 480)
        )

        # LIDAR
        self.lidar = LidarRtx(
            prim_path="/World/Robot/base_link/Lidar",
            translation=np.array([0.1, 0.0, 0.2]),
            config="Example_Rotary_Mechanical_Lidar",
            rotation_frequency=10,
            samples_per_scan=1000,
            update_frequency=10
        )

    def run_simulation(self):
        """Run the simulation and collect sensor data"""
        self.world.reset()

        for i in range(200):
            self.world.step(render=True)

            if i % 30 == 0:  # Log every 30 steps
                rgb_image = self.camera.get_rgb()
                depth_image = self.camera.get_depth()
                print(f"Step {i}: RGB shape {rgb_image.shape}, Depth shape {depth_image.shape}")

    def cleanup(self):
        """Clean up the simulation"""
        self.world.clear()

# Run the exercise
if __name__ == "__main__":
    exercise = IsaacSimExercise1()
    exercise.setup_environment()
    exercise.add_robot()
    exercise.add_sensors()
    exercise.run_simulation()
    exercise.cleanup()
```

## Exercise 2: Isaac ROS Perception Pipeline

### Objective
Create an Isaac ROS perception pipeline with GPU-accelerated object detection.

### Requirements
1. Set up Isaac ROS packages for perception
2. Configure GPU-accelerated AprilTag detection
3. Implement CenterPose object detection
4. Optimize the pipeline using NITROS

### Steps
1. **Verify Isaac ROS installation**:
   ```bash
   # Check if Isaac ROS packages are available
   ros2 pkg list | grep isaac_ros
   ```

2. **Create a launch file** for the perception pipeline:
   ```python
   from launch import LaunchDescription
   from launch_ros.actions import ComposableNodeContainer
   from launch_ros.descriptions import ComposableNode

   def generate_launch_description():
       # Create container for composable nodes
       container = ComposableNodeContainer(
           name='perception_container',
           namespace='isaac_ros',
           package='rclcpp_components',
           executable='component_container_mt',
           composable_node_descriptions=[
               # Image format converter for NITROS
               ComposableNode(
                   package='isaac_ros_image_format',
                   plugin='nvidia::isaac_ros::image_format::ImageFormatConverter',
                   name='image_format_converter',
                   parameters=[{
                       'input_encoding': 'rgb8',
                       'output_encoding': 'rgba8'
                   }],
                   remappings=[
                       ('image', '/camera/color/image_raw'),
                       ('output', '/image_converter/output')
                   ]
               ),

               # AprilTag detection node
               ComposableNode(
                   package='isaac_ros_apriltag',
                   plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                   name='apriltag',
                   parameters=[{
                       'size': 0.3,
                       'max_tags': 10,
                       'tag_family': 'tag36h11'
                   }],
                   remappings=[
                       ('image', '/image_converter/output'),
                       ('detections', '/apriltag_detections')
                   ]
               )
           ],
           output='screen',
       )

       return LaunchDescription([container])
   ```

3. **Run the pipeline**:
   ```bash
   # Source your ROS workspace
   source install/setup.bash

   # Launch the perception pipeline
   ros2 launch perception_pipeline.launch.py
   ```

4. **Test the pipeline** with sample data:
   ```bash
   # Echo AprilTag detections
   ros2 topic echo /apriltag_detections
   ```

### Expected Outcome
AprilTag detections should be published with GPU acceleration providing improved performance.

### Solution Template
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Declare launch arguments
    use_camera = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Use camera input'
    )

    # Create composable node container
    perception_container = ComposableNodeContainer(
        name='isaac_ros_perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Image format converter for NITROS optimization
            ComposableNode(
                package='isaac_ros_image_format',
                plugin='nvidia::isaac_ros::image_format::ImageFormatConverter',
                name='image_format_converter',
                parameters=[{
                    'input_encoding': 'rgb8',
                    'output_encoding': 'rgba8',
                    'cuda_device_index': 0
                }],
                remappings=[
                    ('image', '/camera/color/image_raw'),
                    ('output', '/image_format_converter/output')
                ]
            ),

            # Isaac ROS AprilTag detection
            ComposableNode(
                package='isaac_ros_apriltag',
                plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                name='apriltag',
                parameters=[{
                    'size': 0.3,
                    'max_tags': 10,
                    'tag_family': 'tag36h11',
                    'max_tag_id': 5
                }],
                remappings=[
                    ('image', '/image_format_converter/output'),
                    ('detections', '/detections')
                ]
            ),

            # Isaac ROS Resize node for optimization
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                name='resize',
                parameters=[{
                    'output_width': 640,
                    'output_height': 480,
                }],
                remappings=[
                    ('image', '/camera/color/image_raw'),
                    ('camera_info', '/camera/color/camera_info'),
                    ('resize/image', '/resize/image'),
                    ('resize/camera_info', '/resize/camera_info')
                ]
            )
        ],
        output='screen',
    )

    return LaunchDescription([
        use_camera,
        perception_container
    ])
```

## Exercise 3: Isaac Nav2 Configuration

### Objective
Configure Nav2 with Isaac-specific optimizations for GPU-accelerated navigation.

### Requirements
1. Install Nav2 and Isaac navigation extensions
2. Configure GPU-accelerated costmaps
3. Set up Isaac-optimized path planner
4. Test navigation in simulation

### Steps
1. **Install Nav2 packages**:
   ```bash
   sudo apt update
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   sudo apt install ros-humble-isaac-ros-navigation  # if available
   ```

2. **Create Nav2 configuration file** with Isaac optimizations:
   ```yaml
   # isaac_nav2_config.yaml
   bt_navigator:
     ros__parameters:
       use_sim_time: True
       global_frame: "map"
       robot_base_frame: "base_link"
       # Isaac-specific behavior tree
       default_nav_to_pose_bt_xml: "package://nav2_bt_navigator/behavior_trees/nav_to_pose_w_replanning_and_recovery.xml"

   controller_server:
     ros__parameters:
       use_sim_time: True
       controller_frequency: 20.0
       min_x_velocity_threshold: 0.001
       # Isaac MPPI controller
       controller_plugins: ["FollowPath"]
       FollowPath:
         plugin: "nav2_mppi_controller::MPPIController"
         time_steps: 50
         model_dt: 0.05
         batch_size: 2000  # Increased for GPU acceleration
         vx_max: 0.5
         wz_max: 1.0
         # Higher gains enabled by GPU acceleration
         obstacle_cost_gain: 100.0
         penalty_gain: 20.0

   local_costmap:
     local_costmap:
       ros__parameters:
         use_sim_time: True
         update_frequency: 5.0
         publish_frequency: 2.0
         global_frame: "odom"
         robot_base_frame: "base_link"
         rolling_window: true
         width: 6  # Increased for better planning
         height: 6
         resolution: 0.05
         # Isaac-specific plugins
         plugins: ["voxel_layer", "inflation_layer"]
         inflation_layer:
           plugin: "nav2_costmap_2d::InflationLayer"
           cost_scaling_factor: 3.0
           inflation_radius: 0.55
         voxel_layer:
           plugin: "nav2_costmap_2d::VoxelLayer"
           enabled: True
           voxel_size: 0.05
           max_voxels: 20000  # Increased for 3D mapping
   ```

3. **Launch Nav2 with Isaac configuration**:
   ```bash
   ros2 launch nav2_bringup navigation_launch.py \
     use_sim_time:=true \
     params_file:=install/isaac_nav2_config.yaml
   ```

4. **Test navigation** using RViz2:
   ```bash
   ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
   ```

### Expected Outcome
Navigation should run with Isaac-optimized parameters, showing improved performance.

## Exercise 4: Isaac ROS Integration with Navigation

### Objective
Integrate Isaac ROS perception with Nav2 for semantic navigation.

### Requirements
1. Set up Isaac ROS perception pipeline
2. Configure Nav2 to use semantic information
3. Test navigation with object detection
4. Implement obstacle avoidance based on object types

### Steps
1. **Create integrated launch file**:
   ```python
   from launch import LaunchDescription
   from launch.actions import IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch.substitutions import PathJoinSubstitution
   from launch_ros.substitutions import FindPackageShare

   def generate_launch_description():
       return LaunchDescription([
           # Launch Nav2
           IncludeLaunchDescription(
               PythonLaunchDescriptionSource([
                   PathJoinSubstitution([
                       FindPackageShare('nav2_bringup'),
                       'launch',
                       'navigation_launch.py'
                   ])
               ]),
               launch_arguments={
                   'use_sim_time': 'true',
                   'params_file': PathJoinSubstitution([
                       FindPackageShare('my_robot_navigation'),
                       'config',
                       'isaac_nav2_config.yaml'
                   ])
               }.items()
           ),

           # Launch Isaac ROS perception
           IncludeLaunchDescription(
               PythonLaunchDescriptionSource([
                   PathJoinSubstitution([
                       FindPackageShare('my_robot_perception'),
                       'launch',
                       'perception_pipeline.launch.py'
                   ])
               ])
           )
       ])
   ```

2. **Create semantic navigation node**:
   ```python
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image, LaserScan
   from visualization_msgs.msg import MarkerArray
   from geometry_msgs.msg import PoseStamped
   from nav2_msgs.action import NavigateToPose
   from rclpy.action import ActionClient

   class SemanticNavigator(Node):
       def __init__(self):
           super().__init__('semantic_navigator')

           # Subscriptions
           self.detection_sub = self.create_subscription(
               MarkerArray,
               '/object_detections',
               self.detection_callback,
               10
           )

           self.laser_sub = self.create_subscription(
               LaserScan,
               '/scan',
               self.scan_callback,
               10
           )

           # Navigation action client
           self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

           # Semantic costmap publisher
           self.semantic_costmap_pub = self.create_publisher(
               OccupancyGrid,
               '/semantic_costmap',
               10
           )

       def detection_callback(self, msg):
           """Process object detections and update semantic costmap"""
           # Process detected objects
           for marker in msg.markers:
               if marker.type == marker.CUBE:  # Assuming objects are marked as cubes
                   self.update_semantic_costmap(marker)

       def update_semantic_costmap(self, object_marker):
           """Update costmap based on object type and location"""
           # Create semantic costmap based on object properties
           # e.g., assign high cost to certain object types (people, fragile items)
           pass

       def navigate_with_semantics(self, goal_pose):
           """Navigate considering semantic information"""
           # Modify navigation behavior based on detected objects
           # e.g., maintain safe distance from people
           pass
   ```

### Expected Outcome
Robot should navigate while considering semantic information from perception system.

## Exercise 5: Performance Optimization and Benchmarking

### Objective
Optimize Isaac Platform components and benchmark performance improvements.

### Requirements
1. Set up performance monitoring tools
2. Benchmark standard vs Isaac-optimized components
3. Optimize parameters for maximum performance
4. Document performance gains

### Steps
1. **Set up performance monitoring**:
   ```bash
   # Monitor GPU utilization
   watch -n 1 nvidia-smi

   # Monitor ROS 2 topics
   ros2 topic hz /camera/color/image_raw
   ros2 topic hz /scan
   ```

2. **Create performance benchmarking script**:
   ```python
   import time
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image
   import numpy as np

   class PerformanceBenchmark(Node):
       def __init__(self):
           super().__init__('performance_benchmark')

           self.subscription = self.create_subscription(
               Image,
               '/camera/color/image_raw',
               self.image_callback,
               10
           )

           self.frame_times = []
           self.start_time = None
           self.frame_count = 0
           self.max_frames = 100

       def image_callback(self, msg):
           """Measure processing time for each frame"""
           if self.frame_count == 0:
               self.start_time = time.time()

           # Process frame (this would be your actual processing)
           self.process_frame(msg)

           self.frame_times.append(time.time())
           self.frame_count += 1

           if self.frame_count >= self.max_frames:
               self.calculate_performance()
               self.frame_count = 0  # Reset for continuous monitoring

       def process_frame(self, msg):
           """Simulate frame processing"""
           # This would be replaced with actual Isaac ROS processing
           pass

       def calculate_performance(self):
           """Calculate and report performance metrics"""
           if len(self.frame_times) > 1:
               intervals = np.diff(self.frame_times)
               avg_interval = np.mean(intervals)
               fps = 1.0 / avg_interval

               self.get_logger().info(
                   f'Performance: {fps:.2f} FPS, '
                   f'Avg processing time: {avg_interval*1000:.2f} ms'
               )
   ```

3. **Compare standard vs Isaac-optimized**:
   ```bash
   # Test standard OpenCV processing
   ros2 run my_package standard_perception_node

   # Test Isaac ROS processing
   ros2 run my_package isaac_ros_perception_node
   ```

### Expected Outcome
Documented performance improvements with Isaac Platform optimizations.

## Exercise 6: Isaac Sim to Real Robot Transfer

### Objective
Transfer a navigation task from Isaac Sim to a real robot using Isaac Platform tools.

### Requirements
1. Develop navigation in Isaac Sim
2. Validate the same algorithm on real hardware
3. Compare performance and behavior
4. Address sim-to-real transfer challenges

### Steps
1. **Develop in simulation**:
   - Create navigation scenario in Isaac Sim
   - Tune parameters for optimal performance
   - Validate with various obstacles and conditions

2. **Deploy to real robot**:
   - Use same navigation stack on real robot
   - Compare performance metrics
   - Identify and address differences

3. **Analyze reality gap**:
   ```python
   class SimRealComparison:
       def __init__(self):
           self.sim_metrics = {}
           self.real_metrics = {}

       def compare_navigation(self):
           """Compare navigation performance between sim and real"""
           # Collect metrics from both environments
           # Compare path quality, execution time, success rate
           pass

       def identify_gap_sources(self):
           """Identify sources of sim-to-real differences"""
           # Sensor noise differences
           # Physics model discrepancies
           # Environmental variations
           pass
   ```

### Expected Outcome
Understanding of sim-to-real transfer challenges and solutions.

## Assessment Rubric

Each exercise will be assessed based on:

- **Implementation Quality** (25%): Is the solution properly implemented?
- **Isaac Platform Usage** (25%): Are Isaac-specific features properly utilized?
- **Performance Optimization** (25%): Are GPU acceleration and NITROS properly implemented?
- **Validation** (25%): Is the implementation properly validated and tested?

## Submission Guidelines

1. Create a GitHub repository for your Isaac Platform exercises
2. Organize code by exercise number
3. Include a README.md with instructions for running each exercise
4. Include screenshots or logs demonstrating successful execution
5. Document performance measurements and optimizations
6. Submit the repository URL for assessment

## Additional Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest)
- [Isaac ROS Tutorials](https://nvidia-isaac-ros.github.io/repositories_and_packages.html)
- [Nav2 Documentation](https://navigation.ros.org/)
- [NVIDIA Robotics Developer Kit](https://developer.nvidia.com/isaac-ros-gems)

These exercises provide hands-on experience with the NVIDIA Isaac Platform, preparing you for advanced robotics applications in subsequent modules.