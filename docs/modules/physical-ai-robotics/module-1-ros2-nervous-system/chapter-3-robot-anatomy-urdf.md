---
title: Understanding Robot Anatomy with URDF
sidebar_label: Chapter 3 - Robot Anatomy & URDF
description: Exploring robot anatomy with URDF, the purpose of URDF in humanoid robotics, defining links, joints, and sensors, and how URDF enables simulation and deployment
---

# Understanding Robot Anatomy with URDF

## Introduction to URDF in Humanoid Robotics

Unified Robot Description Format (URDF) is the standard XML-based format for describing robot models in ROS. In humanoid robotics, URDF defines the physical structure, kinematic properties, and sensor configuration of robots with human-like morphology.

### Key Benefits of URDF for Humanoid Robots

- **Kinematic Modeling**: Defines joint arrangements for human-like movement
- **Simulation Support**: Provides accurate physical models for realistic simulation
- **Standardization**: Common format for sharing robot models across the ROS ecosystem
- **Tool Integration**: Compatible with RViz, Gazebo, MoveIt, and other ROS tools

## Purpose of URDF in Humanoid Robotics

URDF serves multiple critical functions in humanoid robotics:

### 1. Structural Definition
- Defines the physical components (links) of the robot
- Specifies how components are connected (joints)
- Establishes the kinematic chain from base to end effectors

### 2. Physical Properties
- Mass and inertial properties of each link
- Visual and collision geometry
- Material properties for simulation

### 3. Sensor Integration
- Mounting positions for cameras, IMUs, and other sensors
- Field of view and specifications for sensors
- Calibration parameters

## Defining Links, Joints, and Sensors

### Links

Links represent the rigid bodies of a robot. In humanoid robots, these correspond to body parts:

```xml
<link name="torso">
  <inertial>
    <mass value="5.0" />
    <origin xyz="0 0 0.2" />
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1" />
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <mesh filename="package://humanoid_description/meshes/torso.dae" />
    </geometry>
    <material name="light_grey">
      <color rgba="0.7 0.7 0.7 1.0" />
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <mesh filename="package://humanoid_description/meshes/torso_collision.dae" />
    </geometry>
  </collision>
</link>
```

### Joints

Joints define how links connect and move relative to each other. For humanoid robots, these include:

- **Revolute joints**: Rotational joints like elbows and knees
- **Continuous joints**: Continuously rotating joints like shoulders
- **Fixed joints**: Non-moving connections
- **Prismatic joints**: Linear motion joints

```xml
<joint name="left_shoulder_pitch" type="revolute">
  <parent link="torso" />
  <child link="left_upper_arm" />
  <origin xyz="0.0 0.15 0.4" rpy="0 0 0" />
  <axis xyz="1 0 0" />
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0" />
  <dynamics damping="0.1" friction="0.0" />
</joint>
```

### Sensors

Sensors are typically defined as fixed joints with sensor plugins:

```xml
<joint name="camera_joint" type="fixed">
  <parent link="head" />
  <child link="camera_link" />
  <origin xyz="0.05 0 0.05" rpy="0 0 0" />
</joint>

<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.05 0.02" />
    </geometry>
  </visual>
</link>

<gazebo reference="camera_link">
  <sensor type="camera" name="head_camera">
    <update_rate>30.0</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Modeling Humanoid Structure and Movement Constraints

Humanoid robots require careful consideration of human-like movement patterns:

### Anthropomorphic Design Principles

1. **Proportional Relationships**: Maintaining realistic ratios between body segments
2. **Range of Motion**: Ensuring joints have appropriate limits based on human anatomy
3. **Degrees of Freedom**: Providing sufficient DOF for complex movements
4. **Center of Mass**: Properly positioned for stable locomotion

### Common Humanoid Joint Configurations

#### Bipedal Locomotion System
- **Hips**: 3 DOF (pitch, roll, yaw) for balance and movement
- **Knees**: 1 DOF (pitch) for walking motion
- **Ankles**: 2 DOF (pitch, roll) for balance and terrain adaptation

#### Upper Body Manipulation System
- **Shoulders**: 3 DOF for wide workspace coverage
- **Elbows**: 1-2 DOF depending on design
- **Wrists**: 2-3 DOF for fine manipulation
- **Hands**: Multiple DOF for grasping

## How URDF Enables Simulation and Real-World Deployment

### Simulation Benefits

1. **Gazebo Integration**: URDF models work directly with Gazebo physics engine
2. **RViz Visualization**: Real-time 3D visualization of robot state
3. **Motion Planning**: Tools like MoveIt use URDF for collision checking
4. **Controller Testing**: Test control algorithms in simulation before deployment

### Deployment Advantages

1. **Kinematic Solvers**: Forward and inverse kinematics based on URDF structure
2. **Collision Detection**: Real-time collision checking using URDF models
3. **Robot State Publishing**: Robot State Publisher uses URDF to broadcast transforms
4. **Hardware Abstraction**: Same URDF works for simulation and real robot

## Practical URDF Example: Simple Humanoid Model

Here's a simplified URDF example showing the basic structure of a humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base Link -->
  <link name="base_link">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01" />
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <inertial>
      <mass value="5.0" />
      <origin xyz="0 0 0.2" />
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.2" />
      <geometry>
        <box size="0.2 0.3 0.4" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" />
      <geometry>
        <box size="0.2 0.3 0.4" />
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 0.05" />
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.05" />
      <geometry>
        <sphere radius="0.1" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" />
      <geometry>
        <sphere radius="0.1" />
      </geometry>
    </collision>
  </link>

  <!-- Joints connecting the body parts -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link" />
    <child link="torso" />
    <origin xyz="0 0 0.3" />
  </joint>

  <joint name="head_joint" type="revolute">
    <parent link="torso" />
    <child link="head" />
    <origin xyz="0 0 0.4" />
    <axis xyz="0 1 0" />
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1.0" />
  </joint>

  <!-- Additional links and joints for arms, legs would follow -->

</robot>
```

### Enhanced Example: Humanoid Arm with Actuators and Sensors

Here's a more detailed example of a humanoid arm with actuators and sensors:

```xml
<!-- Left Arm -->
<link name="left_upper_arm">
  <inertial>
    <mass value="2.0" />
    <origin xyz="0 0 0.15" />
    <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.005" />
  </inertial>
  <visual>
    <origin xyz="0 0 0.15" />
    <geometry>
      <capsule radius="0.05" length="0.2" />
    </geometry>
    <material name="blue">
      <color rgba="0.2 0.2 0.8 1.0" />
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0.15" />
    <geometry>
      <capsule radius="0.05" length="0.2" />
    </geometry>
  </collision>
</link>

<link name="left_lower_arm">
  <inertial>
    <mass value="1.5" />
    <origin xyz="0 0 0.12" />
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.003" />
  </inertial>
  <visual>
    <origin xyz="0 0 0.12" />
    <geometry>
      <capsule radius="0.04" length="0.16" />
    </geometry>
    <material name="blue" />
  </visual>
  <collision>
    <origin xyz="0 0 0.12" />
    <geometry>
      <capsule radius="0.04" length="0.16" />
    </geometry>
  </collision>
</link>

<!-- Joint with actuator properties -->
<joint name="left_elbow_joint" type="revolute">
  <parent link="left_upper_arm" />
  <child link="left_lower_arm" />
  <origin xyz="0 0 0.3" />
  <axis xyz="0 1 0" />
  <limit lower="-2.356" upper="2.356" effort="100" velocity="5.0" />
  <dynamics damping="0.5" friction="0.1" />
</joint>

<!-- IMU sensor example -->
<joint name="left_lower_arm_imu_joint" type="fixed">
  <parent link="left_lower_arm" />
  <child link="left_lower_arm_imu" />
  <origin xyz="0 0 0.15" rpy="0 0 0" />
</joint>

<link name="left_lower_arm_imu">
  <inertial>
    <mass value="0.01" />
    <origin xyz="0 0 0" />
    <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001" />
  </inertial>
</link>

<!-- Gazebo plugin for the IMU -->
<gazebo reference="left_lower_arm_imu">
  <sensor type="imu" name="left_arm_imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

### Visual Representation of Humanoid Kinematic Chain

Here's a text-based diagram showing the kinematic structure of a humanoid robot:

```
                    head
                     |
                  neck_joint
                     |
                    torso
                   /   \
                 /       \
         left_shoulder   right_shoulder
              |             |
         left_upper_arm   right_upper_arm
              |             |
         left_elbow       right_elbow
              |             |
         left_lower_arm   right_lower_arm
              |             |
         left_wrist       right_wrist
              |             |
         left_hand        right_hand

             torso
               |
         waist_joint
               |
             pelvis
            /      \
          /          \
   left_hip       right_hip
        |              |
   left_upper_leg  right_upper_leg
        |              |
   left_knee       right_knee
        |              |
   left_lower_leg  right_lower_leg
        |              |
   left_ankle      right_ankle
        |              |
   left_foot       right_foot
```

*Accessibility Note: This diagram shows the hierarchical structure of a humanoid robot, with the head at the top connected to the torso, which branches into arms and connects down to the pelvis and legs. The kinematic chain flows from torso to head, arms, and legs.*

## Best Practices for URDF Development

1. **Use Xacro**: For complex humanoid robots, use Xacro macros to avoid repetition
2. **Proper Inertial Properties**: Accurate mass and inertia values for stable simulation
3. **Collision vs Visual**: Separate collision and visual meshes for performance
4. **Transform Hierarchy**: Maintain proper parent-child relationships
5. **Joint Limits**: Set realistic limits based on physical constraints
6. **Gazebo Plugins**: Include necessary plugins for sensors and actuators

## Cross-References to Related Topics

To understand how URDF models integrate with the broader ROS 2 ecosystem, consider these related topics:

- **ROS 2 Fundamentals**: See [Chapter 1](./chapter-1-ros2-fundamentals.md) for how URDF models interact with nodes, topics, and services
- **AI Integration**: Review [Chapter 2](./chapter-2-ai-agents-ros2.md) to understand how AI agents use URDF models for planning and control
- **Simulation**: URDF models are essential for robot simulation in environments like Gazebo
- **Motion Planning**: URDF provides the kinematic structure needed for path planning algorithms in MoveIt

## Summary

URDF is fundamental to humanoid robotics in ROS, providing the structural definition that enables both simulation and real-world deployment. Understanding how to properly define links, joints, and sensors in URDF is essential for creating functional humanoid robots that can be simulated, controlled, and deployed effectively.

## Key Takeaways

- URDF serves as the digital blueprint for humanoid robots in ROS
- Properly defined links and joints enable realistic movement and simulation
- Sensor integration in URDF allows for perception capabilities
- URDF models work across simulation and real-world deployment
- Best practices ensure stable and efficient robot models

## Exercises

1. **Modeling Practice**: Design a simplified URDF for a 6-DOF robotic arm, including appropriate links, joints, and visual/collision geometry.

2. **Kinematic Analysis**: Given a humanoid leg URDF with hip, knee, and ankle joints, describe how you would calculate the foot position from joint angles.

3. **Simulation Integration**: Explain how you would modify a URDF model to include Gazebo plugins for a camera sensor and joint controllers.

---

**Previous**: [Chapter 2 - AI Agents & ROS2](./chapter-2-ai-agents-ros2.md) | **Next**: [Glossary](./glossary.md)