# Sensor Data Processing Exercises

## Exercise 1: LiDAR Obstacle Detection

### Objective
Process LiDAR scan data to detect obstacles and map the environment.

### Instructions
1. Subscribe to the `/humanoid_robot/lidar_scan` topic
2. Process the LaserScan messages to identify obstacles
3. Filter out noise and unreliable readings
4. Group nearby points into obstacle clusters
5. Estimate obstacle positions and sizes

### Steps
1. Create a subscriber node for LaserScan messages
2. Implement a function to identify valid range readings
3. Apply filters to remove noise (e.g., remove ranges beyond max range)
4. Implement clustering algorithm to group nearby points
5. Calculate bounding boxes or centroids for each obstacle cluster

### Expected Output
List of obstacles with their positions and approximate sizes.

### Solution Reference
```python
def process_lidar_scan(self, scan_msg):
    obstacles = []

    # Process each ray in the scan
    for i, r in enumerate(scan_msg.ranges):
        if not math.isnan(r) and scan_msg.range_min < r < scan_msg.range_max:
            # Convert polar to Cartesian
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            # Only consider obstacles within certain distance
            if 0.2 < r < 3.0:
                obstacles.append({'x': x, 'y': y, 'distance': r})

    # Cluster nearby obstacles
    clustered_obstacles = self.cluster_obstacles(obstacles)
    return clustered_obstacles
```

---

## Exercise 2: IMU-Based Motion Estimation

### Objective
Estimate robot motion using IMU data.

### Instructions
1. Subscribe to the `/humanoid_robot/imu/data` topic
2. Process linear acceleration to estimate velocity
3. Integrate velocity to estimate position
4. Account for drift and calibration offsets

### Steps
1. Create a subscriber node for IMU messages
2. Implement numerical integration for velocity and position
3. Apply filtering to reduce drift
4. Calibrate sensor offsets
5. Compare estimated motion with ground truth (if available)

### Expected Output
Estimated robot trajectory based on IMU measurements.

### Solution Reference
```python
def process_imu(self, imu_msg):
    # Get linear acceleration (remove gravity component if needed)
    acc_x = imu_msg.linear_acceleration.x
    acc_y = imu_msg.linear_acceleration.y
    acc_z = imu_msg.linear_acceleration.z

    # Numerical integration to get velocity and position
    dt = 0.01  # Time step

    # Update velocity (v = v0 + a*t)
    self.velocity_x += acc_x * dt
    self.velocity_y += acc_y * dt
    self.velocity_z += acc_z * dt

    # Update position (p = p0 + v*t)
    self.pos_x += self.velocity_x * dt
    self.pos_y += self.velocity_y * dt
    self.pos_z += self.velocity_z * dt
```

---

## Exercise 3: Camera-Based Object Detection

### Objective
Detect and classify objects in camera images.

### Instructions
1. Subscribe to the `/humanoid_robot/camera/color/image_raw` topic
2. Convert ROS image to OpenCV format
3. Implement object detection algorithm
4. Draw bounding boxes around detected objects

### Steps
1. Create a subscriber node for Image messages
2. Use cv_bridge to convert ROS Image to OpenCV format
3. Implement basic computer vision techniques (e.g., color-based detection)
4. For advanced users: Use pre-trained neural networks for object detection
5. Publish results as annotated images or object positions

### Expected Output
Image with bounding boxes drawn around detected objects.

### Solution Reference
```python
def camera_callback(self, img_msg):
    # Convert ROS Image message to OpenCV format
    cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

    # Convert to HSV for color-based detection
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Define color range (e.g., for detecting red objects)
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])

    # Create mask
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw bounding boxes
    for contour in contours:
        if cv2.contourArea(contour) > 100:  # Minimum area threshold
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
```

---

## Exercise 4: Multi-Sensor Fusion

### Objective
Combine data from multiple sensors for improved environmental perception.

### Instructions
1. Subscribe to LiDAR, camera, and IMU topics simultaneously
2. Process each sensor's data individually
3. Fuse the data to create a more accurate environmental model
4. Account for sensor uncertainties and timing differences

### Steps
1. Create a node that subscribes to multiple sensor topics
2. Use message_filters to synchronize messages from different sensors
3. Implement a fusion algorithm (e.g., Kalman filter, particle filter)
4. Validate that fused data is more accurate than individual sensors

### Expected Output
Enhanced environmental perception that combines multiple sensor inputs.

### Solution Reference
```python
import message_filters

def __init__(self):
    # Create subscribers
    lidar_sub = message_filters.Subscriber(self, LaserScan, '/lidar_scan')
    imu_sub = message_filters.Subscriber(self, Imu, '/imu/data')

    # Synchronize messages
    ts = message_filters.ApproximateTimeSynchronizer([lidar_sub, imu_sub],
                                                     queue_size=10, slop=0.1)
    ts.registerCallback(self.sync_callback)

def sync_callback(self, lidar_msg, imu_msg):
    # Process both messages together
    lidar_data = self.process_lidar(lidar_msg)
    imu_data = self.process_imu(imu_msg)

    # Fuse the data
    fused_result = self.fuse_sensors(lidar_data, imu_data)
```

---

## Exercise 5: Sensor Calibration

### Objective
Calibrate sensor parameters to improve accuracy.

### Instructions
1. Collect sensor data in known environments
2. Identify systematic errors in sensor readings
3. Calculate calibration parameters
4. Apply corrections to sensor data

### Steps
1. Collect LiDAR data from known distances (e.g., walls at known positions)
2. Measure discrepancy between actual and measured distances
3. Fit a calibration curve to correct measurements
4. Apply the correction in your sensor processing pipeline

### Expected Output
Improved sensor accuracy after calibration.

### Solution Reference
```python
def calibrate_lidar(self, measured_distances, actual_distances):
    # Fit a linear calibration curve: corrected = a * measured + b
    coefficients = np.polyfit(measured_distances, actual_distances, 1)
    a, b = coefficients

    def apply_calibration(distance):
        return a * distance + b

    return apply_calibration
```

---

## Exercise 6: Sensor Noise Analysis

### Objective
Analyze sensor noise characteristics and implement filtering.

### Instructions
1. Collect sensor data in stationary conditions
2. Analyze the noise distribution
3. Implement appropriate filtering techniques
4. Compare raw vs. filtered sensor data

### Steps
1. Record IMU data while robot is stationary
2. Plot histogram of accelerometer readings
3. Calculate mean and standard deviation of noise
4. Implement a low-pass filter to reduce noise
5. Compare performance of different filter types

### Expected Output
Filtered sensor data with reduced noise compared to raw measurements.

### Solution Reference
```python
def apply_low_pass_filter(self, new_value, alpha=0.1):
    # Exponential moving average: y[n] = alpha*x[n] + (1-alpha)*y[n-1]
    if self.filtered_value is None:
        self.filtered_value = new_value
    else:
        self.filtered_value = alpha * new_value + (1 - alpha) * self.filtered_value

    return self.filtered_value
```

---

## Exercise 7: Real-Time Sensor Processing

### Objective
Implement efficient sensor processing for real-time applications.

### Instructions
1. Process sensor data within timing constraints
2. Optimize algorithms for computational efficiency
3. Monitor processing time and system resources
4. Implement fallback strategies when processing falls behind

### Steps
1. Profile your sensor processing code to identify bottlenecks
2. Optimize algorithms (e.g., reduce image resolution, use faster algorithms)
3. Implement message throttling if needed
4. Add processing time monitoring and logging

### Expected Output
Real-time sensor processing that maintains required update rates.

### Solution Reference
```python
def process_sensor_data(self, msg):
    start_time = time.time()

    # Process sensor data here
    result = self.expensive_computation(msg)

    processing_time = time.time() - start_time
    self.get_logger().debug(f'Sensor processing took {processing_time:.3f}s')

    # Warn if processing time exceeds acceptable threshold
    if processing_time > 0.05:  # 50ms threshold
        self.get_logger().warn('Sensor processing is taking too long!')
```

---

## Exercise 8: Sensor Validation and Testing

### Objective
Create tests to validate sensor processing algorithms.

### Instructions
1. Create synthetic sensor data for testing
2. Write unit tests for sensor processing functions
3. Validate sensor outputs against expected values
4. Test edge cases and error conditions

### Steps
1. Generate synthetic LiDAR data with known obstacles
2. Test your obstacle detection algorithm with this data
3. Write tests for different scenarios (no obstacles, multiple obstacles, noisy data)
4. Create a test suite that validates your sensor processing pipeline

### Expected Output
Comprehensive test suite that validates sensor processing functionality.

### Solution Reference
```python
import unittest

class TestSensorProcessing(unittest.TestCase):
    def setUp(self):
        self.processor = SensorProcessor()

    def test_obstacle_detection(self):
        # Create synthetic LiDAR data with known obstacle
        scan_msg = LaserScan()
        scan_msg.ranges = [1.0, 1.0, 0.5, 0.5, 1.0]  # Obstacle at 0.5m
        scan_msg.angle_increment = 0.1
        scan_msg.angle_min = -0.2

        obstacles = self.processor.detect_obstacles_from_lidar(scan_msg)

        # Verify obstacle is detected
        self.assertEqual(len(obstacles), 1)
        self.assertLess(obstacles[0]['distance'], 0.6)  # Should detect obstacle at 0.5m
```

---

## Validation Checklist

For each exercise, verify:
- [ ] Sensor data is received correctly
- [ ] Processing algorithm runs without errors
- [ ] Outputs are reasonable and expected
- [ ] Code handles edge cases gracefully
- [ ] Performance meets real-time requirements (if applicable)
- [ ] Documentation is clear and complete
- [ ] Tests pass successfully
- [ ] Algorithm accuracy is validated