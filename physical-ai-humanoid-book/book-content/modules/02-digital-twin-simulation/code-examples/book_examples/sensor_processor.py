#!/usr/bin/env python3
"""
Sensor Processor for the Physical AI & Humanoid Robotics Book

This node processes sensor data from the simulated humanoid robot.
Demonstrates handling of LiDAR, camera, and IMU sensor data.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
from cv_bridge import CvBridge
import numpy as np
import math
from geometry_msgs.msg import Vector3


class SensorProcessor(Node):
    """
    Node that processes data from multiple sensors on the humanoid robot
    """
    def __init__(self):
        super().__init__('sensor_processor')

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

        # Publisher for processed sensor data
        self.processed_data_pub = self.create_publisher(
            Vector3,  # Using Vector3 as a simple data container
            '/processed_sensor_data',
            10
        )

        # Store sensor data for analysis
        self.latest_lidar_data = None
        self.latest_image_data = None
        self.latest_imu_data = None

        # Statistics for sensor data
        self.lidar_stats = {
            'min_distance': float('inf'),
            'max_distance': 0,
            'avg_distance': 0,
            'num_scans': 0
        }

        # Timer for periodic processing
        self.processing_timer = self.create_timer(1.0, self.process_sensors)

        self.get_logger().info('Sensor Processor node initialized')

    def lidar_callback(self, msg):
        """
        Process LiDAR scan data
        """
        self.latest_lidar_data = msg

        # Update statistics
        valid_ranges = [r for r in msg.ranges if not math.isnan(r) and r > 0]
        if valid_ranges:
            self.lidar_stats['min_distance'] = min(valid_ranges)
            self.lidar_stats['max_distance'] = max(valid_ranges)
            self.lidar_stats['avg_distance'] = sum(valid_ranges) / len(valid_ranges)
            self.lidar_stats['num_scans'] = len(valid_ranges)

        self.get_logger().debug(f'LiDAR: {len(msg.ranges)} ranges, min={self.lidar_stats["min_distance"]:.2f}m')

    def camera_callback(self, msg):
        """
        Process camera image data
        """
        self.latest_image_data = msg

        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform basic image analysis
            height, width, channels = cv_image.shape
            avg_brightness = np.mean(cv_image)

            self.get_logger().debug(f'Camera: {width}x{height} @ {avg_brightness:.2f} avg brightness')
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')

    def imu_callback(self, msg):
        """
        Process IMU data
        """
        self.latest_imu_data = msg

        # Extract orientation and angular velocity
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration

        # Calculate magnitude of vectors
        ang_vel_mag = math.sqrt(
            angular_velocity.x**2 +
            angular_velocity.y**2 +
            angular_velocity.z**2
        )

        lin_acc_mag = math.sqrt(
            linear_acceleration.x**2 +
            linear_acceleration.y**2 +
            linear_acceleration.z**2
        )

        self.get_logger().debug(f'IMU: orient=({orientation.x:.2f}, {orientation.y:.2f}), ang_vel_mag={ang_vel_mag:.2f}, lin_acc_mag={lin_acc_mag:.2f}')

    def process_sensors(self):
        """
        Periodic processing of all sensor data
        """
        self.get_logger().info('Processing sensor data...')

        # Publish processed data summary
        summary_msg = Vector3()

        if self.latest_lidar_data:
            summary_msg.x = self.lidar_stats['avg_distance']  # Average distance from obstacles
        else:
            summary_msg.x = -1.0  # Indicate no data

        if self.latest_imu_data:
            # Use magnitude of linear acceleration as indicator of movement
            lin_acc = self.latest_imu_data.linear_acceleration
            summary_msg.y = math.sqrt(lin_acc.x**2 + lin_acc.y**2 + lin_acc.z**2)
        else:
            summary_msg.y = -1.0  # Indicate no data

        # Placeholder for image analysis result
        summary_msg.z = 0.0  # Could represent number of detected objects, etc.

        self.processed_data_pub.publish(summary_msg)

        # Log sensor status
        lidar_status = "ACTIVE" if self.latest_lidar_data else "INACTIVE"
        camera_status = "ACTIVE" if self.latest_image_data else "INACTIVE"
        imu_status = "ACTIVE" if self.latest_imu_data else "INACTIVE"

        self.get_logger().info(f'Sensor Status - LiDAR: {lidar_status}, Camera: {camera_status}, IMU: {imu_status}')

    def get_environment_map(self):
        """
        Create a simple occupancy grid from LiDAR data
        """
        if not self.latest_lidar_data:
            return None

        # Simplified occupancy grid based on LiDAR ranges
        # In a real implementation, this would be more sophisticated
        ranges = self.latest_lidar_data.ranges
        angle_increment = self.latest_lidar_data.angle_increment
        angle_min = self.latest_lidar_data.angle_min

        # Create a simplified representation of obstacles
        obstacles = []
        for i, r in enumerate(ranges):
            if not math.isnan(r) and 0 < r < 5.0:  # Only nearby obstacles
                angle = angle_min + i * angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                obstacles.append((x, y))

        return obstacles

    def detect_movement(self):
        """
        Detect if the robot is moving based on IMU data
        """
        if not self.latest_imu_data:
            return False

        # Check if linear acceleration exceeds threshold (indicating movement)
        lin_acc = self.latest_imu_data.linear_acceleration
        threshold = 0.5  # m/s²
        mag = math.sqrt(lin_acc.x**2 + lin_acc.y**2 + lin_acc.z**2)

        # Account for gravity (approximately 9.8 m/s² on z-axis)
        net_acc = abs(mag - 9.8)

        return net_acc > threshold


def main(args=None):
    """
    Main function to initialize and run the sensor processor
    """
    rclpy.init(args=args)

    node = SensorProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()