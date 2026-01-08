# ROS 2 Node Communication Exercises

## Exercise 1: Basic Publisher/Subscriber

### Objective
Create a simple publisher that sends messages and a subscriber that receives them.

### Instructions
1. Create a publisher node that sends a counter value every second
2. Create a subscriber node that receives and prints the counter value
3. Launch both nodes simultaneously and observe the communication

### Steps
1. Create a new package called `exercise1_pkg`
2. Create a publisher node that publishes `std_msgs/String` messages to topic `/counter`
3. Create a subscriber node that subscribes to `/counter` and logs received messages
4. Build and run both nodes using `ros2 run`
5. Verify that messages are being transmitted correctly

### Expected Output
Subscriber should log increasing counter values approximately every second.

### Solution Reference
See `book_examples/publisher_member_function.py` and `book_examples/subscriber_member_function.py` for reference implementation.

---

## Exercise 2: Parameterized Publisher

### Objective
Extend the publisher to accept parameters for message content and publishing frequency.

### Instructions
1. Modify the publisher from Exercise 1 to accept parameters
2. Use `declare_parameter()` to set default values
3. Allow runtime configuration of message content and frequency

### Steps
1. Add parameters for message text and publishing interval
2. Use `get_parameter()` to retrieve parameter values
3. Test with different parameter values using command line arguments

### Expected Output
Publisher should adapt its behavior based on parameters.

### Solution Reference
```python
# In your node constructor:
self.declare_parameter('message_text', 'Default message')
self.declare_parameter('publish_freq', 1.0)

message_text = self.get_parameter('message_text').value
freq = self.get_parameter('publish_freq').value
```

---

## Exercise 3: Joint State Publisher

### Objective
Create a publisher that simulates joint states for a humanoid robot.

### Instructions
1. Publish `sensor_msgs/JointState` messages
2. Include positions, velocities, and efforts for multiple joints
3. Make the joint positions vary sinusoidally over time

### Steps
1. Create a publisher for `/joint_states`
2. Define joint names for a humanoid (e.g., 'hip_joint', 'knee_joint', 'shoulder_joint')
3. Generate time-varying positions for each joint
4. Include corresponding velocities and efforts

### Expected Output
Should see realistic joint position values changing smoothly over time.

### Solution Reference
```python
msg = JointState()
msg.name = ['left_hip', 'left_knee', 'right_hip', 'right_knee']
msg.position = [math.sin(t), math.sin(t + 0.5), math.sin(t + 1.0), math.sin(t + 1.5)]
msg.velocity = [math.cos(t), math.cos(t + 0.5), math.cos(t + 1.0), math.cos(t + 1.5)]
msg.effort = [0.0, 0.0, 0.0, 0.0]
```

---

## Exercise 4: Service Server and Client

### Objective
Create a service that calculates the distance between two points and a client that calls it.

### Instructions
1. Define a custom service that takes two 3D points and returns the distance
2. Implement the service server
3. Create a client that sends requests to the server

### Steps
1. Create a service definition file `Distance.srv`:
   ```
   geometry_msgs/Point point1
   geometry_msgs/Point point2
   ---
   float64 distance
   ```
2. Implement server that calculates Euclidean distance
3. Implement client that sends test points

### Expected Output
Client should receive correct distance calculations from the server.

### Solution Reference
```python
# Server callback
def calc_distance(self, request, response):
    dx = request.point1.x - request.point2.x
    dy = request.point1.y - request.point2.y
    dz = request.point1.z - request.point2.z
    response.distance = math.sqrt(dx*dx + dy*dy + dz*dz)
    return response
```

---

## Exercise 5: Action Server for Robot Movement

### Objective
Create an action server that simulates moving a robot to a target position.

### Instructions
1. Create an action that takes a target pose and provides feedback on progress
2. Implement the action server with feedback and result
3. Create a client that sends goals to the action server

### Steps
1. Define an action file `MoveToPose.action`:
   ```
   # Goal
   geometry_msgs/Pose target_pose
   ---
   # Result
   bool success
   string message
   ---
   # Feedback
   float32 percent_complete
   geometry_msgs/Pose current_pose
   ```
2. Implement action server that simulates movement
3. Implement action client that sends goals

### Expected Output
Client should receive feedback about progress and final result when goal is reached.

### Solution Reference
See `book_examples/walk_action_server.py` and `book_examples/walk_action_client.py` for reference implementation.

---

## Exercise 6: Multi-Node Coordination

### Objective
Coordinate multiple nodes to achieve a joint task (e.g., controlling multiple joints).

### Instructions
1. Create multiple publisher nodes that control different aspects of the robot
2. Create a coordinator node that orchestrates the others
3. Use services for coordination signals

### Steps
1. Create separate nodes for left arm, right arm, and legs
2. Create a coordinator node that sends commands to each
3. Use services to signal readiness between nodes

### Expected Output
All nodes should work in coordination to achieve the desired robot behavior.

---

## Exercise 7: Quality of Service (QoS) Tuning

### Objective
Experiment with different QoS settings to understand their impact on communication.

### Instructions
1. Create a publisher with different QoS profiles
2. Create subscribers with matching and mismatched QoS profiles
3. Observe how QoS affects message delivery

### Steps
1. Publish with reliable vs. best-effort QoS
2. Subscribe with volatile vs. transient-local durability
3. Compare message delivery under different conditions

### Expected Output
Reliable QoS should ensure all messages are delivered; best-effort may drop messages under stress.

---

## Exercise 8: Launch File Configuration

### Objective
Create launch files to manage complex multi-node systems.

### Instructions
1. Create a launch file that starts multiple nodes for robot control
2. Use parameters in the launch file
3. Launch the system and verify all nodes start correctly

### Steps
1. Create a launch directory in your package
2. Write a Python launch file that starts your nodes
3. Include parameters and remappings as needed

### Expected Output
All nodes should start simultaneously with correct parameters when launch file is executed.

### Solution Reference
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='publisher_node',
            name='my_publisher',
            parameters=[{'param_name': 'param_value'}]
        ),
        Node(
            package='your_package',
            executable='subscriber_node',
            name='my_subscriber'
        )
    ])
```

---

## Validation Checklist

For each exercise, verify:
- [ ] Nodes start without errors
- [ ] Communication occurs as expected
- [ ] Messages are received correctly
- [ ] Error handling is appropriate
- [ ] Code follows ROS 2 conventions
- [ ] Documentation is clear and complete