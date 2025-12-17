---
title: Glossary of Terms
sidebar_label: Glossary
description: Definitions of key terms used in ROS 2, rclpy, and URDF concepts for humanoid robotics
---

# Glossary of Terms

## A

### Actuation
The process of converting control signals into physical movement or action by robot actuators (motors, servos, etc.).

### Action
A ROS 2 concept that provides goal-oriented communication between nodes, with feedback and status updates for long-running tasks.

## C

### Collision Geometry
The geometric representation of a robot link used for collision detection in simulation and planning.

## G

### Gazebo
A physics-based 3D simulation environment that integrates with ROS for robot simulation and testing.

## J

### Joint
A connection between two links in a robot that allows relative motion. Types include revolute, prismatic, continuous, and fixed joints.

## L

### Link
A rigid body component of a robot in URDF, representing a physical part like a body segment or sensor mount.

## M

### Middleware
Software that provides common services and capabilities to applications beyond what's offered by the operating system, specifically referring to ROS 2 in robotics.

### MoveIt
A motion planning framework for ROS that provides tools for path planning, inverse kinematics, and collision checking.

## N

### Node
A process that performs computation in ROS. Nodes are the fundamental building blocks of ROS applications and communicate with each other through topics, services, and actions.

## P

### Publisher
A ROS entity that sends messages to a topic in the publish-subscribe communication model.

### Publish-Subscribe Model
A communication pattern where publishers send messages to topics without knowing which subscribers will receive them.

## R

### rclpy
The Python client library for ROS 2, providing the interface between Python programs and the ROS 2 middleware.

### Robot Operating System 2 (ROS 2)
The next generation of the Robot Operating System, providing libraries and tools to help write robot software, including communication, hardware abstraction, and development tools.

### ROS 2
Short for Robot Operating System 2, the distributed computing framework for robotics applications.

## S

### Service
A ROS communication pattern that provides synchronous request-response interaction between nodes.

### Subscriber
A ROS entity that receives messages from a topic in the publish-subscribe communication model.

## T

### Topic
A ROS communication channel over which nodes exchange messages in a publish-subscribe model.

## U

### Unified Robot Description Format (URDF)
An XML format for representing a robot model, including links, joints, and their relationships, used for robot visualization and simulation.

### URDF
Short for Unified Robot Description Format, the standard XML-based format for describing robot models in ROS.

## V

### Visual Geometry
The geometric representation of a robot link used for visualization in tools like RViz.

## X

### Xacro
An XML macro language that extends URDF, allowing for reuse of common URDF elements and expressions for complex robot models.

---

[Previous](./chapter-3-robot-anatomy-urdf.md) | [Next](./module-overview.md)