---
sidebar_position: 6
---

# Module 2: Practical Exercises

## Exercise 1: Gazebo Environment Setup

### Objective
Set up a basic Gazebo environment with a simple robot model and test basic functionality.

### Requirements
1. Install Gazebo and ROS 2 integration packages
2. Create a simple differential drive robot model
3. Design a basic environment with obstacles
4. Test robot spawning and basic control

### Steps
1. **Install Gazebo packages**:
   ```bash
   sudo apt update
   sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
   ```

2. **Create robot URDF** with:
   - Base link
   - Two wheels
   - Differential drive transmission
   - Basic visual and collision properties

3. **Create a world file** with:
   - Ground plane
   - Simple obstacles
   - Lighting configuration

4. **Launch the simulation**:
   ```bash
   # Terminal 1
   ros2 launch gazebo_ros empty_world.launch.py world:=path/to/your_world.sdf

   # Terminal 2
   ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_robot
   ```

5. **Test basic movement** by publishing to cmd_vel topic

### Expected Outcome
Robot should spawn successfully in Gazebo and respond to velocity commands.

### Solution Template
```xml
<!-- simple_robot.urdf -->
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <link name="wheel_right">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0 0.15 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="0 -0.15 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

## Exercise 2: Sensor Integration in Gazebo

### Objective
Add and configure sensors on your robot model in Gazebo.

### Requirements
1. Add a camera sensor to the robot
2. Add a LIDAR sensor to the robot
3. Configure sensor parameters appropriately
4. Test sensor data publication in ROS 2

### Steps
1. **Modify the URDF** to include sensor plugins
2. **Configure camera parameters** (resolution, FOV, etc.)
3. **Configure LIDAR parameters** (range, resolution, etc.)
4. **Launch simulation and verify** sensor data publication
5. **Visualize sensor data** using ROS 2 tools

### Expected Outcome
Robot should have functioning camera and LIDAR sensors publishing data to ROS 2 topics.

### Sensor Plugin Example
```xml
<!-- Add to robot URDF -->
<gazebo reference="base_link">
  <!-- Camera sensor -->
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
      <min_depth>0.1</min_depth>
      <max_depth>10.0</max_depth>
    </plugin>
  </sensor>

  <!-- LIDAR sensor -->
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
</gazebo>
```

## Exercise 3: Unity Robot Setup

### Objective
Set up a Unity environment for robotics simulation and import a robot model.

### Requirements
1. Install Unity with Robotics packages
2. Import a URDF robot model into Unity
3. Configure physics properties
4. Set up ROS communication

### Steps
1. **Install Unity Hub and Editor** (2021.3 LTS or later)
2. **Install Unity Robotics packages** via Package Manager
3. **Import URDF model** using URDF Importer
4. **Configure physics** properties for the robot
5. **Set up ROS-TCP-Connector** for ROS 2 communication

### Expected Outcome
Robot model should be properly imported in Unity with physics and ROS communication configured.

### Unity Setup Code
```csharp
// RobotController.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.MessageTypes;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    string cmdVelTopic = "cmd_vel";

    [Header("Robot Configuration")]
    public float wheelRadius = 0.1f;
    public float wheelSeparation = 0.5f;

    private GameObject leftWheel, rightWheel;
    private float leftWheelVelocity = 0f, rightWheelVelocity = 0f;

    void Start()
    {
        ros = ROSConnection.instance;
        ros.Subscribe<TwistMsg>(cmdVelTopic, ReceiveVelocityCommand);

        // Find wheel objects in the imported URDF model
        leftWheel = transform.Find("wheel_left")?.gameObject;
        rightWheel = transform.Find("wheel_right")?.gameObject;
    }

    void ReceiveVelocityCommand(TwistMsg cmd)
    {
        // Convert linear and angular velocities to wheel velocities
        float linearVel = cmd.linear.x;
        float angularVel = cmd.angular.z;

        float leftVel = (linearVel - angularVel * wheelSeparation / 2) / wheelRadius;
        float rightVel = (linearVel + angularVel * wheelSeparation / 2) / wheelRadius;

        leftWheelVelocity = leftVel;
        rightWheelVelocity = rightVel;
    }

    void FixedUpdate()
    {
        // Apply wheel velocities using Unity physics
        if (leftWheel != null)
        {
            var leftRb = leftWheel.GetComponent<Rigidbody>();
            if (leftRb != null)
            {
                leftRb.angularVelocity = new Vector3(0, 0, leftWheelVelocity);
            }
        }
        if (rightWheel != null)
        {
            var rightRb = rightWheel.GetComponent<Rigidbody>();
            if (rightRb != null)
            {
                rightRb.angularVelocity = new Vector3(0, 0, rightWheelVelocity);
            }
        }
    }
}
```

## Exercise 4: Physics Parameter Tuning

### Objective
Tune physics parameters to achieve realistic robot behavior in both Gazebo and Unity.

### Requirements
1. Adjust friction coefficients for realistic movement
2. Tune contact properties for stable simulation
3. Optimize solver parameters for performance
4. Validate behavior against expected physical properties

### Steps
1. **Test robot movement** with default physics parameters
2. **Adjust friction values** to match real-world behavior
3. **Tune contact stiffness and damping**
4. **Optimize solver iterations** for stability
5. **Compare behavior** between simulation and expected real-world performance

### Expected Outcome
Robot should move realistically with appropriate friction and contact responses.

### Parameter Tuning Example
```python
import numpy as np

class PhysicsParameterTuner:
    def __init__(self):
        self.parameters = {
            'friction_coeff': 0.8,
            'restitution': 0.1,
            'linear_damping': 0.1,
            'angular_damping': 0.1,
            'solver_iterations': 10,
            'contact_surface_layer': 0.001
        }

    def test_robot_movement(self, params):
        """Simulate robot movement and return metrics"""
        # This would interface with the physics simulator
        # Return metrics like stability, accuracy, performance
        # Implementation would depend on the specific simulator
        pass

    def optimize_parameters(self):
        """Optimize parameters using grid search or other method"""
        best_params = None
        best_score = float('inf')

        # Example: Grid search over friction coefficients
        for friction in np.linspace(0.1, 1.0, 10):
            params = self.parameters.copy()
            params['friction_coeff'] = friction

            score = self.evaluate_parameters(params)
            if score < best_score:
                best_score = score
                best_params = params

        return best_params

    def evaluate_parameters(self, params):
        """Evaluate parameter set based on multiple criteria"""
        stability = self.test_stability(params)
        accuracy = self.test_accuracy(params)
        performance = self.test_performance(params)

        # Weighted combination of metrics
        score = 0.4 * stability + 0.4 * accuracy + 0.2 * performance
        return score
```

## Exercise 5: Sensor Simulation and Validation

### Objective
Implement realistic sensor models and validate them against expected behavior.

### Requirements
1. Create camera sensor model with realistic noise
2. Implement LIDAR sensor with appropriate error characteristics
3. Add IMU sensor with bias and drift
4. Validate sensor outputs against ground truth

### Steps
1. **Implement camera model** with distortion and noise
2. **Create LIDAR simulation** with range limitations and noise
3. **Model IMU sensors** with bias and drift characteristics
4. **Validate outputs** by comparing to ground truth data
5. **Analyze sensor performance** under different conditions

### Expected Outcome
Sensors should produce realistic data with appropriate noise and error characteristics.

### Sensor Validation Example
```python
import numpy as np

class SensorValidator:
    def __init__(self):
        self.ground_truth = None  # True values from simulation
        self.measurements = []    # Sensor measurements

    def validate_camera(self, true_position, measured_image):
        """Validate camera sensor against ground truth"""
        # Calculate expected image features based on true position
        expected_features = self.calculate_expected_features(true_position)

        # Compare with measured features
        measured_features = self.extract_features(measured_image)

        # Calculate error metrics
        error = np.mean((expected_features - measured_features)**2)
        return error

    def validate_lidar(self, true_obstacles, measured_scan):
        """Validate LIDAR sensor against ground truth"""
        # Compare measured ranges to expected ranges
        expected_ranges = self.calculate_expected_ranges(true_obstacles)

        # Calculate error metrics
        range_errors = np.abs(measured_scan - expected_ranges)
        mean_error = np.mean(range_errors)
        std_error = np.std(range_errors)

        return mean_error, std_error

    def validate_imu(self, true_motion, measured_imu):
        """Validate IMU sensor against ground truth"""
        # Compare measured acceleration/gyro to true values
        accel_error = np.mean((measured_imu['accel'] - true_motion['accel'])**2)
        gyro_error = np.mean((measured_imu['gyro'] - true_motion['gyro'])**2)

        return accel_error, gyro_error
```

## Exercise 6: Multi-Simulator Comparison

### Objective
Compare robot behavior and sensor outputs between Gazebo and Unity simulations.

### Requirements
1. Implement identical robot in both simulators
2. Run identical scenarios in both simulators
3. Compare sensor outputs and robot behaviors
4. Identify and analyze differences

### Steps
1. **Create equivalent robot models** in both simulators
2. **Design identical test scenarios** (e.g., navigation, obstacle avoidance)
3. **Run experiments** in both simulators with same inputs
4. **Collect and compare** sensor data and robot states
5. **Analyze differences** and identify causes

### Expected Outcome
Understanding of differences between simulators and their impact on robot behavior.

### Comparison Framework
```python
import pandas as pd
import matplotlib.pyplot as plt

class SimulatorComparison:
    def __init__(self):
        self.gazebo_data = pd.DataFrame()
        self.unity_data = pd.DataFrame()

    def run_comparison_test(self, scenario):
        """Run identical test in both simulators"""
        # Run test in Gazebo
        gazebo_results = self.run_gazebo_test(scenario)

        # Run test in Unity
        unity_results = self.run_unity_test(scenario)

        # Store results
        self.gazebo_data = gazebo_results
        self.unity_data = unity_results

    def analyze_differences(self):
        """Analyze differences between simulators"""
        # Calculate differences for key metrics
        pos_diff = self.gazebo_data['position'] - self.unity_data['position']
        sensor_diff = self.gazebo_data['sensor'] - self.unity_data['sensor']

        # Statistical analysis
        mean_pos_diff = pos_diff.mean()
        std_pos_diff = pos_diff.std()

        return {
            'position_error_mean': mean_pos_diff,
            'position_error_std': std_pos_diff,
            'sensor_error_mean': sensor_diff.mean(),
            'sensor_error_std': sensor_diff.std()
        }

    def visualize_comparison(self):
        """Create plots comparing both simulators"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))

        # Position comparison
        axes[0,0].plot(self.gazebo_data['time'], self.gazebo_data['position'], label='Gazebo')
        axes[0,0].plot(self.unity_data['time'], self.unity_data['position'], label='Unity')
        axes[0,0].set_title('Position Comparison')
        axes[0,0].legend()

        # Sensor comparison
        axes[0,1].plot(self.gazebo_data['time'], self.gazebo_data['sensor'], label='Gazebo')
        axes[0,1].plot(self.unity_data['time'], self.unity_data['sensor'], label='Unity')
        axes[0,1].set_title('Sensor Comparison')
        axes[0,1].legend()

        # Error over time
        error = self.gazebo_data['sensor'] - self.unity_data['sensor']
        axes[1,0].plot(self.gazebo_data['time'], error)
        axes[1,0].set_title('Sensor Error Over Time')

        # Histogram of errors
        axes[1,1].hist(error, bins=50)
        axes[1,1].set_title('Error Distribution')

        plt.tight_layout()
        plt.show()
```

## Exercise 7: Simulation-to-Reality Transfer

### Objective
Test the effectiveness of simulation in preparing for real-world deployment.

### Requirements
1. Develop robot controller in simulation
2. Deploy same controller on real robot (or with real robot data)
3. Compare performance between simulation and reality
4. Identify and address reality gap issues

### Steps
1. **Develop controller** in simulation environment
2. **Test controller** thoroughly in simulation
3. **Deploy on real robot** (or use real robot data)
4. **Compare performance** metrics
5. **Analyze differences** and refine simulation

### Expected Outcome
Understanding of simulation-to-reality transfer challenges and solutions.

## Assessment Rubric

Each exercise will be assessed based on:

- **Implementation Quality** (30%): Is the solution properly implemented?
- **Physics Accuracy** (25%): Are physics parameters realistic and appropriate?
- **Sensor Modeling** (25%): Are sensors modeled with realistic characteristics?
- **Validation** (20%): Is the implementation properly validated?

## Submission Guidelines

1. Create a GitHub repository for your simulation exercises
2. Organize code by exercise number
3. Include a README.md with instructions for running each exercise
4. Include screenshots or logs demonstrating successful execution
5. Document parameter values and validation results
6. Submit the repository URL for assessment

## Additional Resources

- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS 2 Navigation](https://navigation.ros.org/)
- [Physics Simulation Best Practices](https://arxiv.org/abs/2008.12761)

These exercises provide hands-on experience with simulation environments, preparing you for more complex robotics applications in subsequent modules.