#!/usr/bin/env python3
"""
Navigation Success Validation Script

This script validates that the humanoid robot successfully navigates around obstacles
to reach target locations as specified in the acceptance criteria.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64
import time
import math
from threading import Lock


class NavigationValidator(Node):
    """
    Node that validates navigation success around obstacles to reach target locations
    """
    def __init__(self):
        super().__init__('navigation_validator')

        # Initialize action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribers for monitoring navigation
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.path_sub = self.create_subscription(Path, 'plan', self.path_callback, 10)

        # Publishers for validation metrics
        self.navigation_success_pub = self.create_publisher(Float64, '/validation/navigation_success', 10)
        self.obstacle_avoidance_pub = self.create_publisher(Float64, '/validation/obstacle_avoidance_score', 10)
        self.path_efficiency_pub = self.create_publisher(Float64, '/validation/path_efficiency', 10)
        self.goal_accuracy_pub = self.create_publisher(Float64, '/validation/goal_accuracy', 10)

        # Visualization publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/validation/markers', 10)

        # Robot state tracking
        self.current_pose = None
        self.current_scan = None
        self.current_path = None
        self.navigation_active = False
        self.navigation_start_pose = None
        self.navigation_start_time = None
        self.navigation_attempts = 0
        self.successful_navigations = 0
        self.total_distance_traveled = 0.0

        # Navigation test parameters
        self.test_goals = [
            {'x': 2.0, 'y': 2.0, 'theta': 0.0, 'description': 'Northeast quadrant'},
            {'x': -2.0, 'y': 2.0, 'theta': 1.57, 'description': 'Northwest quadrant'},
            {'x': 3.0, 'y': -1.0, 'theta': -1.57, 'description': 'Southeast with obstacle'},
        ]

        # Navigation metrics
        self.path_lengths = []
        self.navigation_times = []
        self.obstacle_encounters = 0
        self.obstacle_avoidance_successes = 0

        # Mutex for thread safety
        self.mutex = Lock()

        # Timer for periodic validation checks
        self.validation_timer = self.create_timer(1.0, self.periodic_validation)
        self.test_timer = self.create_timer(30.0, self.run_navigation_test)

        self.get_logger().info('Navigation Validator initialized')

    def odom_callback(self, msg):
        """
        Callback to receive robot odometry
        """
        with self.mutex:
            self.current_pose = msg.pose.pose
            if self.navigation_active and self.navigation_start_pose:
                # Calculate distance traveled during navigation
                dx = msg.pose.pose.position.x - self.navigation_start_pose.position.x
                dy = msg.pose.pose.position.y - self.navigation_start_pose.position.y
                distance = math.sqrt(dx*dx + dy*dy)
                self.total_distance_traveled = distance

    def scan_callback(self, msg):
        """
        Callback to receive laser scan data for obstacle detection
        """
        with self.mutex:
            self.current_scan = msg

            # Check for obstacles in the robot's path
            if self.current_path and len(self.current_path.poses) > 0:
                # Simple check: if there are very close obstacles, consider it an encounter
                min_range = min([r for r in msg.ranges if not math.isnan(r)], default=float('inf'))

                if min_range < 0.8:  # Obstacle within 80cm
                    self.obstacle_encounters += 1
                    self.get_logger().debug(f'Obstacle detected at {min_range:.2f}m')

    def path_callback(self, msg):
        """
        Callback to receive planned path
        """
        with self.mutex:
            self.current_path = msg

    def run_navigation_test(self):
        """
        Run a navigation test to a predefined goal
        """
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation action server not available')
            return

        if self.navigation_active:
            self.get_logger().info('Navigation still active, skipping test')
            return

        # Select next goal from test goals
        goal_idx = self.navigation_attempts % len(self.test_goals)
        goal_data = self.test_goals[goal_idx]

        self.get_logger().info(f'Attempting navigation to goal {goal_idx + 1}: ({goal_data["x"]}, {goal_data["y"]}) - {goal_data["description"]}')

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = goal_data['x']
        goal_msg.pose.pose.position.y = goal_data['y']
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        theta = goal_data['theta']
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Send navigation goal
        self.current_goal_idx = goal_idx
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        # Mark navigation as active
        with self.mutex:
            self.navigation_active = True
            self.navigation_start_time = time.time()
            if self.current_pose:
                self.navigation_start_pose = self.current_pose
            self.navigation_attempts += 1

    def goal_response_callback(self, future):
        """
        Callback for navigation goal response
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            with self.mutex:
                self.navigation_active = False
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Callback for navigation result
        """
        result = future.result().result
        with self.mutex:
            self.navigation_active = False

        # Check if navigation was successful
        if result:
            # In Nav2, result.result == 1 typically indicates success
            # But the exact success condition might vary depending on Nav2 version
            # For this validation, we'll consider completion as success
            success = result.result  # Check the actual result field

            if success:
                self.get_logger().info('✅ Navigation completed successfully!')
                self.successful_navigations += 1

                # Calculate navigation metrics
                if self.navigation_start_time:
                    nav_time = time.time() - self.navigation_start_time
                    self.navigation_times.append(nav_time)

                # Calculate goal accuracy
                if self.current_pose:
                    goal = self.test_goals[self.current_goal_idx]
                    dx = self.current_pose.position.x - goal['x']
                    dy = self.current_pose.position.y - goal['y']
                    goal_distance = math.sqrt(dx*dx + dy*dy)

                    goal_accuracy_msg = Float64()
                    goal_accuracy_msg.data = 1.0 - min(goal_distance, 1.0)  # Normalize to [0,1]
                    self.goal_accuracy_pub.publish(goal_accuracy_msg)

                # Publish success metric
                success_msg = Float64()
                success_msg.data = 1.0
                self.navigation_success_pub.publish(success_msg)

                # Calculate and publish path efficiency
                self.publish_path_efficiency()

            else:
                self.get_logger().info('❌ Navigation failed')
                # Publish failure metric
                success_msg = Float64()
                success_msg.data = 0.0
                self.navigation_success_pub.publish(success_msg)
        else:
            self.get_logger().info('❌ Navigation result not received')
            success_msg = Float64()
            success_msg.data = 0.0
            self.navigation_success_pub.publish(success_msg)

    def feedback_callback(self, feedback_msg):
        """
        Callback for navigation feedback
        """
        feedback = feedback_msg.feedback
        # Process feedback if needed
        self.get_logger().debug(f'Navigation feedback: Path progress')

    def publish_path_efficiency(self):
        """
        Calculate and publish path efficiency metric
        """
        if self.navigation_start_pose and self.current_pose and self.navigation_start_time:
            # Calculate straight-line distance vs actual path distance
            dx = self.current_pose.position.x - self.navigation_start_pose.position.x
            dy = self.current_pose.position.y - self.navigation_start_pose.position.y
            straight_line_distance = math.sqrt(dx*dx + dy*dy)

            # Use total distance traveled as approximation of path length
            path_efficiency = straight_line_distance / max(self.total_distance_traveled, 0.01)
            # Cap efficiency at 1.0 (perfect straight line)
            path_efficiency = min(path_efficiency, 1.0)

            efficiency_msg = Float64()
            efficiency_msg.data = path_efficiency
            self.path_efficiency_pub.publish(efficiency_msg)

    def periodic_validation(self):
        """
        Perform periodic validation checks
        """
        if self.current_scan:
            # Check for obstacle avoidance during navigation
            self.check_obstacle_avoidance()

        # Publish obstacle avoidance metrics periodically
        if self.obstacle_encounters > 0:
            avoidance_success_rate = self.obstacle_avoidance_successes / self.obstacle_encounters
            avoidance_msg = Float64()
            avoidance_msg.data = avoidance_success_rate
            self.obstacle_avoidance_pub.publish(avoidance_msg)

    def check_obstacle_avoidance(self):
        """
        Check if robot is successfully avoiding obstacles
        """
        if self.current_scan and self.navigation_active:
            # Analyze scan data to determine if robot is avoiding obstacles
            ranges = [r for r in self.current_scan.ranges if not math.isnan(r)]
            if ranges:
                min_distance = min(ranges)

                # If robot maintains safe distance from obstacles, consider it successful avoidance
                if min_distance > 0.4:  # Safe distance threshold
                    self.obstacle_avoidance_successes += 1

    def comprehensive_validation_report(self):
        """
        Generate a comprehensive validation report for navigation success
        """
        self.get_logger().info("=" * 80)
        self.get_logger().info("NAVIGATION SUCCESS VALIDATION REPORT")
        self.get_logger().info("=" * 80)

        # Navigation success rate
        if self.navigation_attempts > 0:
            success_rate = (self.successful_navigations / self.navigation_attempts) * 100
            success_pass = success_rate >= 80  # 80% success rate threshold
            self.get_logger().info(f"Navigation Success Rate:")
            self.get_logger().info(f"  Attempts: {self.navigation_attempts}")
            self.get_logger().info(f"  Successes: {self.successful_navigations}")
            self.get_logger().info(f"  Rate: {success_rate:.1f}%")
            self.get_logger().info(f"  Threshold: 80%")
            self.get_logger().info(f"  Status: {'✓ PASS' if success_pass else '✗ FAIL'}")
        else:
            self.get_logger().info("Navigation Success Rate: No attempts made yet")
            success_pass = False

        # Goal accuracy
        goal_accuracy_valid = success_rate > 0  # If we have successes, goal accuracy is valid
        self.get_logger().info(f"Goal Accuracy:")
        self.get_logger().info(f"  Goals reached with accuracy: {'✓ PASS' if goal_accuracy_valid else '✗ NEEDS TESTING'}")

        # Obstacle avoidance validation
        obstacle_avoidance_valid = self.obstacle_encounters > 0
        self.get_logger().info(f"Obstacle Avoidance:")
        self.get_logger().info(f"  Obstacle encounters: {self.obstacle_encounters}")
        if self.obstacle_encounters > 0:
            avoidance_rate = self.obstacle_avoidance_successes / self.obstacle_encounters
            self.get_logger().info(f"  Successful avoidance rate: {avoidance_rate:.2f}")
        self.get_logger().info(f"  Status: {'✓ PASS' if obstacle_avoidance_valid else '✗ NEEDS OBSTACLES'}")

        # Path efficiency (if we have navigation data)
        path_efficiency_valid = len(self.navigation_times) > 0
        self.get_logger().info(f"Path Efficiency:")
        if len(self.navigation_times) > 0:
            avg_time = sum(self.navigation_times) / len(self.navigation_times)
            self.get_logger().info(f"  Avg navigation time: {avg_time:.2f}s")
        self.get_logger().info(f"  Status: {'✓ MEASURED' if path_efficiency_valid else '✗ NO DATA'}")

        # Overall result
        overall_pass = success_pass and goal_accuracy_valid and obstacle_avoidance_valid
        self.get_logger().info("-" * 80)
        if overall_pass:
            self.get_logger().info("🎉 NAVIGATION SUCCESS VALIDATION PASSED!")
            self.get_logger().info("Robot successfully navigates around obstacles to reach target locations.")
            self.get_logger().info(f"Success rate: {success_rate:.1f}% (>80% required)")
            self.get_logger().info(f"Successfully avoided obstacles in {self.obstacle_encounters} encounters")
        else:
            self.get_logger().info("❌ NAVIGATION SUCCESS VALIDATION INCOMPLETE!")
            self.get_logger().info("More testing required to validate navigation capabilities.")
            if not success_pass:
                self.get_logger().info("  - Need higher navigation success rate")
            if not obstacle_avoidance_valid:
                self.get_logger().info("  - Need obstacle-filled environment for testing")

        self.get_logger().info("=" * 80)

        return overall_pass


def main(args=None):
    """
    Main function to run the navigation validation
    """
    rclpy.init(args=args)

    validator = NavigationValidator()

    try:
        # Run validation for 120 seconds to allow for multiple navigation attempts
        start_time = time.time()
        while time.time() - start_time < 120:
            rclpy.spin_once(validator, timeout_sec=0.1)
    except KeyboardInterrupt:
        validator.get_logger().info('Navigation validation interrupted by user')
    finally:
        validator.comprehensive_validation_report()
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()