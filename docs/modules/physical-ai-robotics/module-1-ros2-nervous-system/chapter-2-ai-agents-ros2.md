---
title: Connecting AI Agents to Robots
sidebar_label: Chapter 2 - AI Agents & ROS2
description: Understanding how AI agents connect to robots using rclpy, bridging decision logic to robot controllers, and the perception-decision-actuation flow
---

# Connecting AI Agents to Robots

## Introduction to rclpy and ROS 2 Nodes

ROS 2 provides Python clients through rclpy, enabling AI agents to interface with robotic hardware. The rclpy library bridges high-level AI decision-making algorithms and low-level robot control systems.

### Key Benefits of rclpy for AI Integration

- **Language Compatibility**: Python is the dominant language for AI and ML development
- **Direct Integration**: Native communication with ROS 2 middleware
- **Real-time Performance**: Efficient message passing between AI algorithms and robot controllers

## Bridging AI Decision Logic to Robot Controllers

The connection between AI agents and robots occurs through ROS 2 nodes that act as intermediaries. These nodes translate high-level goals from AI systems into actionable commands for robot hardware.

### The Bridge Architecture

```
AI Agent (High-Level Goals) → ROS 2 Bridge Node → Robot Controllers (Low-Level Actions)
```

#### Key Components of the Bridge:

1. **Goal Translation Layer**: Converts abstract AI decisions into specific robot commands
2. **Safety Validation**: Ensures AI commands comply with safety constraints
3. **State Monitoring**: Tracks robot state and reports back to AI agent
4. **Fallback Mechanisms**: Handles communication failures gracefully

## Command Flow: Perception → Decision → Actuation

The fundamental cycle of AI-driven robotics operates in a continuous loop:

### 1. Perception Phase

- Sensors collect environmental data (cameras, lidars, IMUs, joint encoders)
- Data is published to ROS 2 topics for AI consumption
- Sensor fusion combines multiple data streams into coherent world models

### 2. Decision Phase

- AI agents consume sensor data from ROS 2 topics
- Machine learning models process information to make decisions
- Decisions are packaged as ROS 2 service calls or topic messages

### 3. Actuation Phase

- Robot controllers receive AI-generated commands via ROS 2
- Commands are translated into motor commands, gripper controls, etc.
- Robot executes actions and publishes feedback to the AI system

## Real-World Applications in Humanoid Control

### Autonomous Navigation

```
Camera/Lidar Data → AI Path Planning → Leg Motor Commands
```

### Object Manipulation

```
Vision Processing → Grasping Strategy → Arm Joint Control
```

### Human-Robot Interaction

```
Speech Recognition → Dialogue Manager → Expressive Behaviors
```

## Conceptual Example Scenarios for Humanoid Behaviors

### Scenario 1: Humanoid Navigation in Dynamic Environments

Consider a humanoid robot that needs to navigate through a crowded space:

```
Perception: Vision and depth sensors detect humans moving in the environment
Decision: AI agent determines safest path around people using social navigation algorithms
Actuation: Leg controllers execute walking gait to follow planned path while avoiding collisions
```

### Scenario 2: Object Manipulation and Grasping

For a humanoid robot picking up objects:

```
Perception: RGB-D camera and tactile sensors detect object position and properties
Decision: AI agent plans grasp strategy based on object shape, weight, and task requirements
Actuation: Arm and hand controllers execute precise joint movements to grasp and manipulate
```

### Scenario 3: Human-Robot Collaboration

In a collaborative task scenario:

```
Perception: Microphones and cameras detect human speech and gestures
Decision: AI agent interprets human intent and determines appropriate response
Actuation: Full-body controllers execute movements that are safe and socially appropriate
```

## Practical Implementation Example: Mobile Robot Control

Here's a simplified example of how an AI agent might control a mobile robot, which can be extended to humanoid systems:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class AIAgentController(Node):
    def __init__(self):
        super().__init__('ai_agent_controller')

        # Publisher for robot velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for laser scan data
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Timer for AI decision-making loop
        self.timer = self.create_timer(0.1, self.ai_decision_loop)

    def scan_callback(self, msg):
        # Process laser scan data for obstacle detection
        self.laser_data = msg

    def ai_decision_loop(self):
        # Simple AI logic: avoid obstacles and move forward
        cmd_msg = Twist()

        if self.is_obstacle_ahead():
            # Turn to avoid obstacle
            cmd_msg.angular.z = 0.5
        else:
            # Move forward
            cmd_msg.linear.x = 0.5

        self.cmd_vel_publisher.publish(cmd_msg)

    def is_obstacle_ahead(self):
        if hasattr(self, 'laser_data'):
            # Check if there are obstacles within 1 meter in front
            front_distances = self.laser_data.ranges[:10] + self.laser_data.ranges[-10:]
            return min(front_distances) < 1.0
        return False

def main():
    rclpy.init()
    ai_controller = AIAgentController()

    try:
        rclpy.spin(ai_controller)
    except KeyboardInterrupt:
        pass
    finally:
        ai_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced Example: Humanoid Upper Body Control

Here's a more complex example showing how an AI agent might control a humanoid robot's upper body for object manipulation:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
import numpy as np

class HumanoidUpperBodyController(Node):
    def __init__(self):
        super().__init__('humanoid_upper_body_controller')

        # Publisher for joint trajectory commands
        self.joint_cmd_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            10
        )

        # Subscriber for target poses from AI agent
        self.target_subscriber = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.target_callback,
            10
        )

        # Current joint state subscriber
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.current_joint_positions = {}
        self.joint_names = ['left_shoulder_pitch', 'left_shoulder_roll',
                           'left_elbow', 'right_shoulder_pitch',
                           'right_shoulder_roll', 'right_elbow']

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_joint_positions[name] = msg.position[i]

    def target_callback(self, msg):
        """Process target pose from AI agent and generate joint trajectory"""
        # This would involve inverse kinematics to convert
        # target pose to joint angles
        target_joint_positions = self.inverse_kinematics(
            msg.pose,
            self.current_joint_positions
        )

        # Create and publish joint trajectory
        trajectory_msg = self.create_trajectory_msg(target_joint_positions)
        self.joint_cmd_publisher.publish(trajectory_msg)

    def inverse_kinematics(self, target_pose, current_joints):
        """Simplified inverse kinematics calculation"""
        # In a real implementation, this would use sophisticated
        # IK algorithms or learning-based approaches
        target_joints = current_joints.copy()

        # Simplified example - in reality this would be complex math
        target_joints['left_shoulder_pitch'] += 0.1
        target_joints['left_elbow'] -= 0.2

        return target_joints

    def create_trajectory_msg(self, joint_positions):
        """Create a joint trajectory message"""
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joint_positions.keys())

        point = JointTrajectoryPoint()
        point.positions = list(joint_positions.values())
        point.time_from_start.sec = 2  # 2 seconds to reach target

        trajectory.points.append(point)
        return trajectory

def main():
    rclpy.init()
    controller = HumanoidUpperBodyController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for AI-Agent Integration

1. **Modular Design**: Keep AI logic separate from ROS 2 communication
2. **Error Handling**: Implement robust error recovery mechanisms
3. **Performance Monitoring**: Track latency between perception and action
4. **Safety First**: Always validate AI commands before execution
5. **Logging**: Maintain detailed logs for debugging and analysis

## Cross-References to Related Topics

To deepen your understanding of AI-agent integration in humanoid robotics, consider these related topics:

- **ROS 2 Fundamentals**: Review [Chapter 1](./chapter-1-ros2-fundamentals.md) for core concepts of nodes, topics, and services that enable AI integration
- **Robot Modeling**: See [Chapter 3](./chapter-3-robot-anatomy-urdf.md) for how URDF models provide the structural foundation for AI control of robot bodies
- **Simulation**: Understanding robot models is essential for testing AI agents in simulation environments like Gazebo
- **Control Theory**: The perception-decision-actuation cycle connects to classical control systems concepts

## Summary

Connecting AI agents to robots through ROS 2 enables sophisticated autonomous behaviors. The rclpy library provides the essential bridge between Python-based AI systems and robot hardware, allowing for seamless integration of perception, decision-making, and actuation. Understanding this connection is crucial for developing intelligent robotic systems that can operate autonomously in complex environments.

## Key Takeaways

- rclpy serves as the bridge between AI agents and ROS 2 robot systems
- The perception-decision-actuation cycle forms the foundation of AI-driven robotics
- Proper safety and validation mechanisms are essential for reliable operation
- Modular design principles help maintain clean separation of concerns

## Exercises

1. **Implementation Challenge**: Extend the example AI agent controller to include obstacle avoidance with multiple sensor inputs (camera and LIDAR).

2. **System Architecture**: Design a ROS 2 node structure for an AI agent that needs to recognize objects, plan grasps, and control a humanoid robot's arms.

3. **Safety Analysis**: Identify three potential safety risks when connecting AI agents to humanoid robots and propose mitigation strategies for each.

---

**Previous**: [Chapter 1 - ROS 2 Fundamentals](./chapter-1-ros2-fundamentals.md) | **Next**: [Chapter 3 - Robot Anatomy with URDF](./chapter-3-robot-anatomy-urdf.md)