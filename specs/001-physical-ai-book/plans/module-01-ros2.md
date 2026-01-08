# Module 01: The Robotic Nervous System (ROS 2)

## Objective
Provide comprehensive educational content covering ROS 2 fundamentals including nodes, topics, services, actions, and how to bridge Python agents to controllers using rclpy. Enable users to understand and implement proper middleware architecture for humanoid robot control.

## Scope / Non-Scope
**In Scope:**
- ROS 2 nodes, topics, services, and actions
- Python agents bridged to controllers using rclpy
- URDF modeling for humanoid robots
- Basic robot control and communication architecture
- Publisher/subscriber patterns and service calls
- Action servers and clients for long-running tasks

**Out of Scope:**
- Advanced ROS 2 distributed computing concepts
- Real-time control systems beyond basic implementation
- Hardware-specific controller interfaces
- Advanced debugging techniques beyond basic troubleshooting

## Key Concepts
- **ROS 2 Nodes**: Independent processes that communicate with each other using topics, services, and actions
- **Topics**: Unidirectional communication channels for streaming data between nodes
- **Services**: Bidirectional request/response communication patterns
- **Actions**: Asynchronous communication for long-running tasks with feedback
- **rclpy**: Python client library for ROS 2
- **URDF**: Unified Robot Description Format for robot modeling
- **Middleware Architecture**: Communication layer that enables decoupled robot system design

## Inputs
- Python programming knowledge
- Basic understanding of robotics concepts
- Development environment with ROS 2 Humble or Iron installed
- Sample robot models or simulation environment

## Outputs
- Understanding of ROS 2 communication patterns
- Working publisher/subscriber node pairs
- Service client/server implementations
- Action client/server implementations
- Basic humanoid robot model in URDF format
- Python agents controlling simulated robot joints

## Tools & Frameworks
- **ROS 2**: Humble Hawksbill or Iron Irwini
- **rclpy**: Python client library for ROS 2
- **URDF**: Unified Robot Description Format
- **RViz2**: Visualization tool for ROS 2
- **Gazebo**: Simulation environment for testing

## Constraints
- All examples must be reproducible and runnable
- Code snippets must follow ROS 2 best practices
- Examples must work with standard ROS 2 distributions
- All implementations must be compatible with humanoid robot models

## Validation Criteria
- Users can create simple publisher/subscriber node pairs that exchange messages successfully
- Users can implement service clients and servers that respond correctly to requests
- Users can create action clients and servers for long-running tasks with feedback
- Users can create valid URDF models for humanoid robots
- All examples compile and run without errors in ROS 2 environment
- Code follows ROS 2 design patterns and conventions