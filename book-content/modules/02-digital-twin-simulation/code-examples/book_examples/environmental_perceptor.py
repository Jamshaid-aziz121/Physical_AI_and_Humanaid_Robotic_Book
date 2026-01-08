#!/usr/bin/env python3
"""
Environmental Perceptor for the Physical AI & Humanoid Robotics Book

This node processes sensor data to perceive the environment around the humanoid robot.
Demonstrates environmental understanding using multiple sensor modalities.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
from geometry_msgs.msg import PointStamped, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import numpy as np
import math
from collections import deque


class EnvironmentalPerceptor(Node):
    """
    Node that processes sensor data to understand the environment around the robot
    """
    def __init__(self):
        super().__init__('environmental_perceptor')

        # Initialize CvBridge for image processing
        self.bridge = CvBridge()

        # Create subscribers for different sensor types
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/humanoid_robot/lidar_scan',
            self.lidar_callback,
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/humanoid_robot/camera/color/image_raw',
            self.camera_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/humanoid_robot/imu/data',
            self.imu_callback,
            10
        )

        # Publishers for perception results
        self.perception_pub = self.create_publisher(
            Vector3,  # Using Vector3 to represent environmental features
            '/environmental_perception',
            10
        )

        self.obstacles_pub = self.create_publisher(
            MarkerArray,
            '/detected_obstacles',
            10
        )

        self.landmarks_pub = self.create_publisher(
            MarkerArray,
            '/detected_landmarks',
            10
        )

        # Store sensor data for temporal analysis
        self.lidar_history = deque(maxlen=10)  # Keep last 10 scans
        self.imu_history = deque(maxlen=10)    # Keep last 10 IMU readings
        self.environment_map = {}              # Detected objects and landmarks

        # Robot state estimation
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.robot_velocity = {'linear': 0.0, 'angular': 0.0}

        # Timer for environmental perception processing
        self.perception_timer = self.create_timer(0.5, self.process_environment)

        self.get_logger().info('Environmental Perceptor node initialized')

    def lidar_callback(self, msg):
        """
        Process LiDAR data to detect obstacles and map environment
        """
        # Store scan in history
        self.lidar_history.append(msg)

        # Process scan to detect obstacles
        obstacles = self.detect_obstacles_from_lidar(msg)

        # Update environment map with detected obstacles
        for obs in obstacles:
            # Simple clustering to group nearby detections
            closest_existing = None
            min_dist = float('inf')

            for key, existing_obs in self.environment_map.items():
                if key.startswith('obstacle_'):
                    dist = math.sqrt((existing_obs['x'] - obs['x'])**2 + (existing_obs['y'] - obs['y'])**2)
                    if dist < min_dist and dist < 0.5:  # Cluster threshold
                        min_dist = dist
                        closest_existing = key

            if closest_existing:
                # Update existing obstacle estimate
                old_x = self.environment_map[closest_existing]['x']
                old_y = self.environment_map[closest_existing]['y']
                # Exponential moving average
                alpha = 0.3
                self.environment_map[closest_existing]['x'] = alpha * obs['x'] + (1 - alpha) * old_x
                self.environment_map[closest_existing]['y'] = alpha * obs['y'] + (1 - alpha) * old_y
            else:
                # Create new obstacle
                obs_id = f'obstacle_{len([k for k in self.environment_map.keys() if k.startswith("obstacle_")])}'
                self.environment_map[obs_id] = obs

    def camera_callback(self, msg):
        """
        Process camera data to detect visual landmarks
        """
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform basic landmark detection (simplified - in reality would use CV techniques)
            height, width = cv_image.shape[:2]

            # Look for distinctive features (simplified detection)
            landmarks = self.detect_visual_landmarks_simple(cv_image)

            # Update environment map with detected landmarks
            for lm in landmarks:
                # Check if landmark already exists (using rough position matching)
                exists = False
                for key, existing_lm in self.environment_map.items():
                    if key.startswith('landmark_'):
                        # Calculate rough distance based on image position
                        # This is a simplification - in reality would use stereo/depth
                        if (abs(existing_lm['img_x'] - lm['img_x']) < 50 and
                            abs(existing_lm['img_y'] - lm['img_y']) < 50):
                            # Update existing landmark
                            alpha = 0.2
                            existing_lm['img_x'] = alpha * lm['img_x'] + (1 - alpha) * existing_lm['img_x']
                            existing_lm['img_y'] = alpha * lm['img_y'] + (1 - alpha) * existing_lm['img_y']
                            exists = True
                            break

                if not exists:
                    # Create new landmark
                    lm_id = f'landmark_{len([k for k in self.environment_map.keys() if k.startswith("landmark_")])}'
                    self.environment_map[lm_id] = lm

        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')

    def imu_callback(self, msg):
        """
        Process IMU data for robot state estimation
        """
        self.imu_history.append(msg)

        # Estimate robot motion from IMU data
        linear_acc = msg.linear_acceleration
        angular_vel = msg.angular_velocity

        # Integrate to estimate velocity and position (simplified)
        dt = 0.01  # Assuming 100Hz IMU rate

        # Update robot state based on IMU data
        self.robot_velocity['linear'] += math.sqrt(linear_acc.x**2 + linear_acc.y**2 + linear_acc.z**2) * dt
        self.robot_velocity['angular'] = math.sqrt(angular_vel.x**2 + angular_vel.y**2 + angular_vel.z**2)

        # Update pose (simplified - in reality would be more complex)
        self.robot_pose['theta'] += self.robot_velocity['angular'] * dt
        self.robot_pose['x'] += self.robot_velocity['linear'] * math.cos(self.robot_pose['theta']) * dt
        self.robot_pose['y'] += self.robot_velocity['linear'] * math.sin(self.robot_pose['theta']) * dt

    def detect_obstacles_from_lidar(self, scan_msg):
        """
        Detect obstacles from LiDAR scan data
        """
        obstacles = []

        # Process each ray in the scan
        for i, r in enumerate(scan_msg.ranges):
            if not math.isnan(r) and scan_msg.range_min < r < scan_msg.range_max:
                # Convert polar to Cartesian
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                # Only consider obstacles within certain distance
                if 0.2 < r < 3.0:  # Ignore very close (robot body) and far objects
                    obstacles.append({
                        'x': x,
                        'y': y,
                        'distance': r,
                        'angle': angle
                    })

        return obstacles

    def detect_visual_landmarks_simple(self, image):
        """
        Detect visual landmarks in the image (simplified implementation)
        """
        landmarks = []

        # Simplified landmark detection (in reality would use feature detection, AprilTags, etc.)
        height, width = image.shape[:2]

        # Detect bright spots as potential landmarks
        try:
            import cv2
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            bright_mask = cv2.inRange(hsv, (0, 0, 200), (180, 50, 255))

            # Find contours of bright regions
            contours, _ = cv2.findContours(bright_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                if cv2.contourArea(contour) > 50:  # Minimum area threshold
                    # Get centroid of the contour
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        landmarks.append({
                            'img_x': cx,
                            'img_y': cy,
                            'confidence': 0.8  # Fixed confidence for simplicity
                        })
        except ImportError:
            # Fallback if OpenCV is not available
            # Just return some dummy landmarks
            landmarks.append({'img_x': 100, 'img_y': 100, 'confidence': 0.5})
            landmarks.append({'img_x': 200, 'img_y': 200, 'confidence': 0.5})

        return landmarks

    def process_environment(self):
        """
        Main environmental perception processing
        """
        self.get_logger().info('Processing environmental perception...')

        # Prepare perception results
        perception_msg = Vector3()

        # Count detected obstacles
        obstacle_count = len([k for k, v in self.environment_map.items() if k.startswith('obstacle_')])
        perception_msg.x = float(obstacle_count)

        # Count detected landmarks
        landmark_count = len([k for k, v in self.environment_map.items() if k.startswith('landmark_')])
        perception_msg.y = float(landmark_count)

        # Estimate openness of environment (inverse of obstacle density)
        if obstacle_count > 0:
            avg_obstacle_distance = sum([v['distance'] for k, v in self.environment_map.items()
                                         if k.startswith('obstacle_') and 'distance' in v]) / obstacle_count
            perception_msg.z = avg_obstacle_distance
        else:
            perception_msg.z = 5.0  # Default if no obstacles detected

        # Publish perception results
        self.perception_pub.publish(perception_msg)

        # Publish visualization markers for obstacles
        self.publish_obstacle_markers()

        # Publish visualization markers for landmarks
        self.publish_landmark_markers()

        self.get_logger().info(f'Environment perception: {obstacle_count} obstacles, {landmark_count} landmarks, avg dist: {perception_msg.z:.2f}m')

    def publish_obstacle_markers(self):
        """
        Publish visualization markers for detected obstacles
        """
        marker_array = MarkerArray()

        for i, (key, obj) in enumerate(self.environment_map.items()):
            if key.startswith('obstacle_'):
                marker = Marker()
                marker.header.frame_id = "base_link"  # Robot-centric frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "obstacles"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                # Position (relative to robot)
                marker.pose.position.x = obj['x']
                marker.pose.position.y = obj['y']
                marker.pose.position.z = 0.5  # Height above ground

                # Orientation (no rotation)
                marker.pose.orientation.w = 1.0

                # Scale
                marker.scale.x = 0.2  # 20cm diameter
                marker.scale.y = 0.2
                marker.scale.z = 0.2

                # Color (red for obstacles)
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8  # Alpha (transparency)

                marker_array.markers.append(marker)

        self.obstacles_pub.publish(marker_array)

    def publish_landmark_markers(self):
        """
        Publish visualization markers for detected landmarks
        """
        marker_array = MarkerArray()

        for i, (key, obj) in enumerate(self.environment_map.items()):
            if key.startswith('landmark_'):
                marker = Marker()
                marker.header.frame_id = "base_link"  # Robot-centric frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "landmarks"
                marker.id = i
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD

                # Position (approximated from image position)
                # This is a simplification - in reality would use depth information
                marker.pose.position.x = obj.get('x', 2.0)  # Default distance
                marker.pose.position.y = obj.get('y', 0.0)  # Default lateral offset
                marker.pose.position.z = 1.0  # Height

                # Orientation (no rotation)
                marker.pose.orientation.w = 1.0

                # Scale
                marker.scale.x = 0.3  # 30cm diameter
                marker.scale.y = 0.3
                marker.scale.z = 0.5  # 50cm height

                # Color (green for landmarks)
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.8  # Alpha (transparency)

                marker_array.markers.append(marker)

        self.landmarks_pub.publish(marker_array)


def main(args=None):
    """
    Main function to initialize and run the environmental perceptor
    """
    rclpy.init(args=args)

    node = EnvironmentalPerceptor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()