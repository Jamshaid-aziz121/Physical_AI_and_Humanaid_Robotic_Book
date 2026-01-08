# Navigation and Perception Exercises

## Exercise 1: Visual SLAM Implementation

### Objective
Implement Visual SLAM (Simultaneous Localization and Mapping) for a humanoid robot.

### Instructions
1. Set up Isaac ROS Visual SLAM node
2. Configure parameters for humanoid robot kinematics
3. Process visual and IMU data to estimate robot pose
4. Build a map of the environment

### Steps
1. Launch Isaac ROS Visual SLAM with your humanoid robot
2. Configure the camera and IMU topics for your robot
3. Adjust parameters for humanoid-specific characteristics (height, stride, etc.)
4. Monitor the pose estimation and map building process
5. Evaluate the quality of localization and mapping

### Expected Output
Real-time pose estimation and environment map building.

### Solution Reference
```python
# Launch Visual SLAM
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam',
                parameters=[{
                    'enable_rectified_edge', True,
                    'enable_fisheye', False,
                    'rectified_images', True,
                    'publish_odom_tf', True,
                    'min_num_features', 100,
                    'max_num_features', 1000,
                    'gpu_occlusion_threshold', 0.001,
                    'gpu_match_threshold', 0.01,
                    'num_tracking_features_out', 100,
                    'min_disparity_threshold', 1.0,
                    'max_disparity_threshold', 100.0,
                    'map_frame', 'map',
                    'odom_frame', 'odom',
                    'base_frame', 'base_link',
                    'init_frame', 'init',
                }],
                remappings=[
                    ('/visual_slam/imu', '/imu/data'),
                    ('/visual_slam/left/camera_info', '/camera/left/camera_info'),
                    ('/visual_slam/right/camera_info', '/camera/right/camera_info'),
                    ('/visual_slam/left/image_rect_color', '/camera/left/image_rect_color'),
                    ('/visual_slam/right/image_rect_color', '/camera/right/image_rect_color'),
                ]
            )
        ],
        output='screen'
    )
    return LaunchDescription([container])
```

---

## Exercise 2: Object Detection and Classification

### Objective
Use Isaac ROS DetectNet to detect and classify objects in the robot's environment.

### Instructions
1. Set up Isaac ROS DetectNet node
2. Configure with appropriate neural network model
3. Process camera images to detect objects
4. Classify detected objects and measure confidence

### Steps
1. Launch DetectNet with a pre-trained model (e.g., ssd_mobilenet_v2_coco)
2. Connect to your robot's camera feed
3. Configure detection parameters (confidence threshold, etc.)
4. Process detections and visualize results
5. Measure detection accuracy and performance

### Expected Output
List of detected objects with bounding boxes, class labels, and confidence scores.

### Solution Reference
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from isaac_ros_detectnet_interfaces.msg import Detection2DArray
from vision_msgs.msg import Detection2D

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_rect_color',
            self.image_callback,
            10
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detectnet/detections',
            self.detection_callback,
            10
        )

        self.get_logger().info('Object Detector initialized')

    def image_callback(self, msg):
        # Process image if needed
        pass

    def detection_callback(self, msg):
        for detection in msg.detections:
            bbox = detection.bbox
            label = detection.results[0].id
            confidence = detection.results[0].score

            self.get_logger().info(
                f'Detected {label} with confidence {confidence:.2f} '
                f'at ({bbox.center.x}, {bbox.center.y})'
            )
```

---

## Exercise 3: Path Planning with Obstacle Avoidance

### Objective
Implement path planning that avoids obstacles in the environment.

### Instructions
1. Set up Nav2 navigation stack for humanoid robot
2. Configure costmap to include obstacle detection
3. Plan paths around static and dynamic obstacles
4. Execute navigation with obstacle avoidance

### Steps
1. Configure Nav2 with humanoid-specific parameters
2. Set up local and global costmaps
3. Implement obstacle detection and costmap updates
4. Test navigation in environments with obstacles
5. Evaluate path efficiency and obstacle avoidance

### Expected Output
Robot successfully navigates to goal while avoiding obstacles.

### Solution Reference
```python
# Relevant parameters in Nav2 configuration
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001

    FollowPath:
      plugin: "nav2_mppi_controller::MPPILocalPlanner"
      max_vel_x: 0.5  # Reduced for humanoid stability
      max_vel_theta: 0.7
      acc_lim_x: 2.5
      acc_lim_theta: 3.2
      xy_goal_tolerance: 0.25
      critics: ["BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
```

---

## Exercise 4: AprilTag Detection and Localization

### Objective
Use Isaac ROS AprilTag to detect fiducial markers for precise localization.

### Instructions
1. Set up Isaac ROS AprilTag node
2. Configure for appropriate tag family
3. Process camera images to detect AprilTags
4. Calculate robot position relative to tags

### Steps
1. Launch AprilTag detector with camera feed
2. Configure tag family and parameters
3. Process tag detections and calculate poses
4. Use tag poses for robot localization
5. Evaluate detection accuracy and pose estimation

### Expected Output
Detected AprilTags with pose information relative to camera frame.

### Solution Reference
```python
# AprilTag launch configuration
composable_node_descriptions=[
    ComposableNode(
        package='isaac_ros_apriltag',
        plugin='isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        parameters=[{
            'family', 'tag36h11',
            'max_tags', 64,
            'quad_decimate', 2.0,
            'quad_sigma', 0.0,
            'refine_edges', 1,
            'decode_sharpening', 0.25,
            'min_tag_width', 0.05,  # meters
        }],
        remappings=[
            ('/image', '/camera/color/image_rect_color'),
            ('/camera_info', '/camera/color/camera_info'),
        ]
    )
]
```

---

## Exercise 5: Multi-Sensor Fusion for Localization

### Objective
Combine data from multiple sensors (camera, IMU, LiDAR) for robust localization.

### Instructions
1. Subscribe to multiple sensor topics
2. Implement sensor fusion algorithm (e.g., Extended Kalman Filter)
3. Combine sensor readings to estimate robot state
4. Compare fused estimate with individual sensor estimates

### Steps
1. Create subscribers for camera, IMU, and LiDAR data
2. Implement fusion algorithm to combine sensor data
3. Estimate robot position, orientation, and velocity
4. Validate fusion results against ground truth (if available)
5. Analyze improvement over individual sensors

### Expected Output
Improved localization estimate compared to individual sensors.

### Solution Reference
```python
import numpy as np
from scipy.spatial.transform import Rotation as R

class SensorFusion:
    def __init__(self):
        # Initialize state vector [x, y, z, qx, qy, qz, qw, vx, vy, vz]
        self.state = np.zeros(10)
        self.covariance = np.eye(10) * 0.1  # Initial uncertainty

    def fuse_camera_imu_lidar(self, camera_data, imu_data, lidar_data):
        # Prediction step using IMU data
        dt = 0.01  # Time step
        self.predict(imu_data, dt)

        # Update step using camera and lidar data
        if camera_data:
            self.update_camera(camera_data)
        if lidar_data:
            self.update_lidar(lidar_data)

        return self.state

    def predict(self, imu_data, dt):
        # Update state based on IMU measurements
        # Simplified prediction model
        self.state[7:10] += np.array([imu_data.ax, imu_data.ay, imu_data.az]) * dt  # velocity
        self.state[0:3] += self.state[7:10] * dt  # position
```

---

## Exercise 6: Free Space Segmentation

### Objective
Implement freespace segmentation to identify traversable areas.

### Instructions
1. Set up Isaac ROS freespace segmentation
2. Process camera and depth data
3. Segment traversable from non-traversable areas
4. Use segmentation for navigation planning

### Steps
1. Launch freespace segmentation node
2. Connect to camera and depth sensor feeds
3. Configure segmentation parameters
4. Process segmentation masks
5. Use masks for path planning and navigation

### Expected Output
Binary mask indicating traversable and non-traversable areas.

### Solution Reference
```python
# Launch file for freespace segmentation
ComposableNode(
    package='isaac_ros_freespace_segmentation',
    plugin='isaac_ros::freespace_segmentation::FreeSpaceSegmentationNode',
    name='freespace_segmentation',
    parameters=[{
        'confidence_threshold', 0.5,
        'max_range', 10.0,
        'min_range', 0.1,
        'grid_size', 100,
        'sensor_mount_height', 1.0,
        'sensor_pitch_angle', 0.0,
    }],
    remappings=[
        ('/freespace_segmentation/camera/image_rect_color', '/camera/color/image_rect_color'),
        ('/freespace_segmentation/camera/camera_info', '/camera/color/camera_info'),
        ('/freespace_segmentation/depth/image_rect', '/camera/depth/image_rect'),
    ]
)
```

---

## Exercise 7: Behavior Tree Navigation

### Objective
Implement complex navigation behaviors using Behavior Trees.

### Instructions
1. Design a behavior tree for navigation tasks
2. Implement nodes for different navigation behaviors
3. Execute navigation with recovery behaviors
4. Handle navigation failures gracefully

### Steps
1. Create a behavior tree XML file for navigation
2. Implement custom behavior tree nodes if needed
3. Test navigation in challenging scenarios
4. Validate recovery behaviors
5. Evaluate overall navigation success rate

### Expected Output
Robust navigation with intelligent recovery from failures.

### Solution Reference
```xml
<!-- Behavior Tree for Navigation with Recovery -->
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence name="NavigateWithRecovery">
            <RecoveryNode number_of_retries="2" name="Recovery">
                <PipelineSequence name="PlanAndExecute">
                    <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
                    <FollowPath path="{path}" controller_id="FollowPath"/>
                </PipelineSequence>
                <ReactiveFallback name="RecoveryFallback">
                    <Spin spin_dist="1.57" name="Spin"/>
                    <Backup backup_dist="0.15" backup_speed="0.025" name="Backup"/>
                    <Wait wait_duration="5" name="Wait"/>
                </ReactiveFallback>
            </RecoveryNode>
        </Sequence>
    </BehaviorTree>
</root>
```

---

## Exercise 8: Dynamic Obstacle Avoidance

### Objective
Implement navigation that can handle moving obstacles.

### Instructions
1. Set up dynamic obstacle detection
2. Modify path planning to account for moving obstacles
3. Implement reactive avoidance behaviors
4. Test navigation in environments with moving obstacles

### Steps
1. Detect moving obstacles using sensor data
2. Predict future positions of obstacles
3. Plan paths that account for obstacle motion
4. Implement reactive avoidance when needed
5. Evaluate performance in dynamic environments

### Expected Output
Robot successfully navigates while avoiding moving obstacles.

### Solution Reference
```python
class DynamicObstacleAvoidance:
    def __init__(self):
        self.moving_obstacles = []  # List of detected moving obstacles
        self.prediction_horizon = 2.0  # seconds

    def predict_obstacle_motion(self, obstacle):
        # Predict future positions of moving obstacles
        predicted_positions = []
        dt = 0.1  # time step
        t = 0
        while t <= self.prediction_horizon:
            pred_pos = obstacle.position + obstacle.velocity * t
            predicted_positions.append(pred_pos)
            t += dt
        return predicted_positions

    def plan_with_dynamic_obstacles(self, goal, moving_obstacles):
        # Plan path considering predicted obstacle positions
        predicted_obstacles = []
        for obs in moving_obstacles:
            predicted = self.predict_obstacle_motion(obs)
            predicted_obstacles.extend(predicted)

        # Use predicted obstacles to modify costmap or path planning
        # Return safe path that avoids predicted obstacle locations
        return self.compute_safe_path(goal, predicted_obstacles)
```

---

## Exercise 9: Semantic Mapping

### Objective
Create semantic maps that include object labels and meanings.

### Instructions
1. Combine object detection with mapping
2. Associate detected objects with map locations
3. Create semantic labels for map regions
4. Use semantic information for navigation decisions

### Steps
1. Run object detection on camera images
2. Associate detections with map coordinates
3. Create semantic map layers
4. Use semantics for navigation planning
5. Evaluate semantic mapping accuracy

### Expected Output
Map with object labels and semantic information for navigation.

### Solution Reference
```python
class SemanticMapper:
    def __init__(self):
        self.semantic_map = {}  # Dictionary of semantic objects in map
        self.object_classifier = None  # Object detection model

    def update_semantic_map(self, detections, robot_pose):
        for detection in detections:
            # Transform detection from camera frame to map frame
            object_pose_map = self.transform_to_map_frame(
                detection.pose, robot_pose
            )

            # Add object to semantic map
            object_info = {
                'class': detection.class_label,
                'confidence': detection.confidence,
                'position': object_pose_map,
                'timestamp': self.get_current_time()
            }

            # Store in semantic map with spatial indexing
            self.add_object_to_map(object_info)

    def get_navigation_constraints(self, goal):
        # Use semantic information to determine navigation constraints
        # e.g., avoid fragile objects, prefer walkways, etc.
        constraints = []

        for obj in self.get_objects_in_path(start, goal):
            if obj['class'] == 'fragile_item':
                constraints.append(('avoid', obj['position']))
            elif obj['class'] == 'walkway':
                constraints.append(('prefer', obj['position']))

        return constraints
```

---

## Exercise 10: Performance Optimization

### Objective
Optimize perception and navigation pipelines for real-time performance.

### Instructions
1. Profile current implementation
2. Identify bottlenecks in perception and navigation
3. Apply optimization techniques
4. Measure performance improvements

### Steps
1. Profile CPU/GPU usage of perception nodes
2. Identify computational bottlenecks
3. Apply optimizations (parallel processing, model compression, etc.)
4. Measure frame rates and latencies
5. Validate that accuracy is maintained

### Expected Output
Optimized pipeline with improved performance while maintaining accuracy.

### Solution Reference
```python
# Example of optimizing perception pipeline
import threading
import queue

class OptimizedPerceptionPipeline:
    def __init__(self):
        self.input_queue = queue.Queue(maxsize=2)
        self.output_queue = queue.Queue(maxsize=2)

        # Use separate threads for different processing stages
        self.preprocessing_thread = threading.Thread(target=self.preprocess_worker)
        self.inference_thread = threading.Thread(target=self.inference_worker)
        self.postprocessing_thread = threading.Thread(target=self.postprocess_worker)

        self.running = True

    def start(self):
        self.preprocessing_thread.start()
        self.inference_thread.start()
        self.postprocessing_thread.start()

    def preprocess_worker(self):
        while self.running:
            try:
                raw_data = self.input_queue.get(timeout=1)
                # Preprocess data (resize, normalize, etc.)
                preprocessed_data = self.preprocess(raw_data)
                self.output_queue.put(preprocessed_data)
            except queue.Empty:
                continue

    def inference_worker(self):
        while self.running:
            try:
                data = self.output_queue.get(timeout=1)
                # Run inference on GPU
                results = self.model_inference(data)
                # Forward to post-processing
            except queue.Empty:
                continue
```

---

## Validation Checklist

For each exercise, verify:
- [ ] Perception system receives sensor data correctly
- [ ] Processing pipeline runs without errors
- [ ] Outputs are reasonable and expected
- [ ] Performance meets real-time requirements
- [ ] Code handles edge cases gracefully
- [ ] Accuracy is validated against ground truth (where available)
- [ ] Documentation is clear and complete
- [ ] Tests pass successfully