"""
Utility functions for robot simulation and control
"""

import math
from typing import List, Tuple, Dict, Any


def calculate_distance(point1: Tuple[float, float, float],
                     point2: Tuple[float, float, float]) -> float:
    """
    Calculate Euclidean distance between two 3D points

    Args:
        point1: (x, y, z) coordinates of first point
        point2: (x, y, z) coordinates of second point

    Returns:
        Distance between the two points
    """
    dx = point1[0] - point2[0]
    dy = point1[1] - point2[1]
    dz = point1[2] - point2[2]
    return math.sqrt(dx*dx + dy*dy + dz*dz)


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-pi, pi] range

    Args:
        angle: Angle in radians

    Returns:
        Normalized angle in [-pi, pi] range
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def quaternion_to_euler(w: float, x: float, y: float, z: float) -> Tuple[float, float, float]:
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)

    Args:
        w, x, y, z: Quaternion components

    Returns:
        Tuple of (roll, pitch, yaw) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """
    Convert Euler angles to quaternion (w, x, y, z)

    Args:
        roll, pitch, yaw: Euler angles in radians

    Returns:
        Tuple of (w, x, y, z) quaternion components
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return w, x, y, z


def clamp(value: float, min_val: float, max_val: float) -> float:
    """
    Clamp a value between min and max values

    Args:
        value: Input value to clamp
        min_val: Minimum allowed value
        max_val: Maximum allowed value

    Returns:
        Clamped value between min_val and max_val
    """
    return max(min_val, min(max_val, value))


class PIDController:
    """
    Simple PID Controller for robot control
    """
    def __init__(self, kp: float, ki: float, kd: float, dt: float = 0.01):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.dt = dt  # Time step

        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, error: float) -> float:
        """
        Update PID controller with current error

        Args:
            error: Current error (setpoint - measurement)

        Returns:
            Control output
        """
        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral += error * self.dt
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / self.dt
        d_term = self.kd * derivative

        # Store current error for next iteration
        self.prev_error = error

        # Calculate output
        output = p_term + i_term + d_term

        return output

    def reset(self):
        """Reset the PID controller"""
        self.prev_error = 0.0
        self.integral = 0.0