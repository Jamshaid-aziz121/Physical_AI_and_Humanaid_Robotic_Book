# ROS 2 Fundamentals for Humanoid Robot Control

## Introduction

Robot Operating System 2 (ROS 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

In this module, we'll explore the core concepts of ROS 2 that are essential for controlling humanoid robots.

## Core ROS 2 Concepts

### Nodes

A node is a process that performs computation. ROS 2 is designed to be modular at the level of a node — each node is responsible for a single purpose (e.g., controlling a laser range finder, publishing odometry data, etc.).

In ROS 2, nodes are contained in packages. This makes nodes more reusable and easier to distribute.

### Topics and Messages

Topics are named buses over which nodes exchange messages. Topics are identified by name and are typed by a message specification.

Messages are data structures that are passed between nodes. They are defined in special `.msg` files and can contain primitive data types as well as other message types.

Communication via topics is asynchronous and follows a publish/subscribe paradigm. Multiple nodes can publish to the same topic, and multiple nodes can subscribe to the same topic.

### Services

Services provide synchronous request/response communication between nodes. A service request is sent from a client to a server, and a response is returned from the server to the client.

Services are defined in `.srv` files and are composed of two parts: a request and a response.

### Actions

Actions are a more advanced communication pattern that allows for long-running tasks with feedback. They consist of three parts:
- Goal: The request to begin a long-running task
- Feedback: Updates about the ongoing action
- Result: The final result of the action

Actions are useful for tasks like navigation, where you want to move to a location but also receive updates about progress and the ability to cancel the action.

## ROS 2 for Humanoid Robot Control

Controlling a humanoid robot requires precise coordination between multiple systems. ROS 2 provides the communication infrastructure to connect these systems:

- **Joint Control**: Publishers and subscribers for sending commands to and receiving feedback from joint controllers
- **Sensor Data**: Topics for sharing data from IMUs, cameras, and other sensors
- **High-Level Planning**: Services and actions for complex behaviors like walking or manipulation
- **Safety Systems**: Monitoring and emergency stop mechanisms

## Practical Examples

### Publisher Example

A publisher node might send joint position commands to control the robot's limbs:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_commands', 10)

    def publish_command(self, joint_positions):
        msg = JointState()
        # ... populate message with joint positions
        self.publisher.publish(msg)
```

### Subscriber Example

A subscriber node might listen to sensor data to monitor the robot's state:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

    def joint_callback(self, msg):
        # Process joint state data
        self.get_logger().info(f'Joint positions: {msg.position}')
```

### Service Example

A service might be used to enable/disable the robot's motors:

```python
# Server
def enable_motors_callback(self, request, response):
    self.motors_enabled = request.data
    response.success = True
    response.message = f'Motors {"enabled" if self.motors_enabled else "disabled"}'
    return response

# Client
def call_enable_motors(self, enable: bool):
    request = SetBool.Request()
    request.data = enable
    future = self.enable_motors_cli.call_async(request)
    # Handle response asynchronously
```

## ROS 2 Architecture

ROS 2 uses a DDS (Data Distribution Service) implementation for communication. This provides:
- **Decentralized**: No central master like in ROS 1
- **Real-time**: Support for real-time systems
- **Security**: Built-in security features
- **Reliability**: Quality of Service (QoS) settings for different communication needs

## Quality of Service (QoS)

QoS settings allow you to tune communication behavior based on your application's requirements:
- **Reliability**: Reliable (all messages delivered) or best-effort (some messages may be dropped)
- **Durability**: Volatile (only new messages) or transient-local (includes historical messages)
- **History**: Keep-all or keep-last N messages

For humanoid robot control, you typically want reliable communication for critical commands and may use best-effort for sensor data where some loss is acceptable.

## Conclusion

Understanding these core ROS 2 concepts is crucial for controlling humanoid robots. The modular nature of ROS 2 allows for complex behaviors to be built from simple, reusable components. In the following sections, we'll dive deeper into implementing these concepts for specific humanoid robot control tasks.