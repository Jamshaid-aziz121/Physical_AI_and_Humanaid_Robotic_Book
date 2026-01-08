#!/bin/bash

# Setup script for Isaac ROS acceleration packages for VSLAM and perception
# This script configures Isaac ROS packages for Visual SLAM and perception acceleration

set -e  # Exit on any error

echo "==============================================="
echo "Setting up Isaac ROS Acceleration for VSLAM and Perception"
echo "==============================================="

# Check if ROS 2 Humble is installed and sourced
if [ -z "$ROS_DISTRO" ] || [ "$ROS_DISTRO" != "humble" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo "Sourced ROS 2 Humble"
    else
        echo "Error: ROS 2 Humble not found. Please install ROS 2 Humble."
        exit 1
    fi
fi

# Create workspace for Isaac ROS packages
ISAAC_ROS_WS="$HOME/isaac_ros_ws"
mkdir -p "$ISAAC_ROS_WS/src"
cd "$ISAAC_ROS_WS"

echo "Created Isaac ROS workspace: $ISAAC_ROS_WS"

# Check if v4l2_camera package is available (needed for Isaac ROS camera support)
if ! dpkg -l | grep -q "v4l2loopback-dkms"; then
    echo "Installing v4l2loopback-dkms for camera support..."
    sudo apt update
    sudo apt install -y v4l2loopback-dkms
fi

# Create colcon build workspace if it doesn't exist
if [ ! -f "src/CMakeLists.txt" ]; then
    cd src
    git clone -b humble https://github.com/ros/ros_tutorials.git
    cd ..
fi

# Create Isaac ROS acceleration configuration
cat << EOF > isaac_ros_acceleration_config.yaml
# Isaac ROS Acceleration Configuration for VSLAM and Perception

isaac_ros:
  version: "humble"
  workspace: "$ISAAC_ROS_WS"

  # VSLAM (Visual Simultaneous Localization and Mapping) packages
  vsland_packages:
    - name: "isaac_ros_visual_slam"
      description: "GPU-accelerated Visual SLAM for pose estimation and map building"
      dependencies:
        - "nvidia-ml-py3"
        - "cuda-toolkit-11-8"  # Adjust based on your CUDA version
      topics:
        input:
          - "camera/color/image_rect_color"
          - "camera/depth/image_rect"
          - "imu/data"
        output:
          - "visual_slam/pose_graph"
          - "visual_slam/odometry"
          - "visual_slam/map"
      parameters:
        enable_debug_mode: false
        enable_localization: true
        enable_mapping: true
        map_frame: "map"
        odom_frame: "odom"
        base_frame: "base_link"
        init_frame: "init"

  # Perception acceleration packages
  perception_packages:
    - name: "isaac_ros_detectnet"
      description: "GPU-accelerated object detection"
      dependencies:
        - "tensorrt"
        - "torch"
      topics:
        input:
          - "camera/color/image_rect_color"
        output:
          - "detectnet/detections"
      parameters:
        model_name: "ssd_mobilenet_v2_coco"
        confidence_threshold: 0.7
        input_tensor: "input_tensor"
        input_format: "nitros_tensor_list"

    - name: "isaac_ros_apriltag"
      description: "GPU-accelerated AprilTag detection"
      topics:
        input:
          - "camera/color/image_rect_color"
        output:
          - "apriltag/detections"
      parameters:
        family: "tag36h11"
        max_tags: 64
        quad_decimate: 2.0
        quad_sigma: 0.0
        refine_edges: 1
        decode_sharpening: 0.25

    - name: "isaac_ros_freespace_segmentation"
      description: "GPU-accelerated freespace segmentation"
      topics:
        input:
          - "camera/color/image_rect_color"
          - "camera/depth/image_rect"
        output:
          - "freespace_segmentation/mask"
      parameters:
        confidence_threshold: 0.5
        max_range: 10.0
        min_range: 0.1

  # Common parameters for all Isaac ROS packages
  common_parameters:
    input_qos:
      history: "keep_last"
      depth: 1
      reliability: "reliable"
      durability: "volatile"
    output_qos:
      history: "keep_last"
      depth: 1
      reliability: "reliable"
      durability: "volatile"

  # Performance settings
  performance:
    cpu_affinity: true
    gpu_affinity: true
    cuda_device: 0
    tensor_rt_cache: "$HOME/.cache/tensorrt"
    memory_pool_size: "100MB"

EOF

echo "Created Isaac ROS acceleration configuration: $ISAAC_ROS_WS/isaac_ros_acceleration_config.yaml"

# Create a setup script for Isaac ROS packages
cat << 'EOF' > setup_isaac_ros_packages.sh
#!/bin/bash

# Script to setup Isaac ROS packages

ISAAC_ROS_WS="$HOME/isaac_ros_ws"
cd $ISAAC_ROS_WS

# Source ROS environment
source /opt/ros/humble/setup.bash

# Clone Isaac ROS packages
cd src

# Visual SLAM package
if [ ! -d "isaac_ros_visual_slam" ]; then
    git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
fi

# DetectNet package
if [ ! -d "isaac_ros_detectnet" ]; then
    git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_detectnet.git
fi

# AprilTag package
if [ ! -d "isaac_ros_apriltag" ]; then
    git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git
fi

# FreeSpace Segmentation package
if [ ! -d "isaac_ros_freespace_segmentation" ]; then
    git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_freespace_segmentation.git
fi

# Additional dependencies
if [ ! -d "isaac_ros_messages" ]; then
    git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_messages.git
fi

if [ ! -d "isaac_ros_common" ]; then
    git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
fi

if [ ! -d "isaac_ros_tensor_list" ]; then
    git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_tensor_list.git
fi

if [ ! -d "isaac_ros_nitros" ]; then
    git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git
fi

cd ..

# Build the workspace
echo "Building Isaac ROS workspace..."
colcon build --symlink-install --packages-select \
    isaac_ros_visual_slam \
    isaac_ros_detectnet \
    isaac_ros_apriltag \
    isaac_ros_freespace_segmentation \
    isaac_ros_messages \
    isaac_ros_common \
    isaac_ros_tensor_list \
    isaac_ros_nitros

# Source the built packages
source install/setup.bash

echo "Isaac ROS packages setup complete!"
echo "To use the packages, source the workspace: source $ISAAC_ROS_WS/install/setup.bash"

EOF

chmod +x setup_isaac_ros_packages.sh
echo "Created Isaac ROS packages setup script: $ISAAC_ROS_WS/setup_isaac_ros_packages.sh"

# Create example launch files for VSLAM
mkdir -p "$ISAAC_ROS_WS/src/isaac_ros_examples/launch"
cat << 'EOF' > "$ISAAC_ROS_WS/src/isaac_ros_examples/launch/visual_slam.launch.py"
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():
    # Visual SLAM parameters
    config_dir = os.path.join(get_package_share_directory('isaac_ros_visual_slam'), 'config')

    return LaunchDescription([
        # Visual SLAM container
        ComposableNodeContainer(
            name='visual_slam_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                # Visual SLAM node
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
    ])
EOF

echo "Created Visual SLAM launch file: $ISAAC_ROS_WS/src/isaac_ros_examples/launch/visual_slam.launch.py"

# Create perception pipeline example
cat << 'EOF' > "$ISAAC_ROS_WS/src/isaac_ros_examples/launch/perception_pipeline.launch.py"
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Perception pipeline container
        ComposableNodeContainer(
            name='perception_pipeline_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                # AprilTag detection node
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
                ),

                # Object detection node
                ComposableNode(
                    package='isaac_ros_detectnet',
                    plugin='isaac_ros::detection::DetectNetNode',
                    name='detectnet',
                    parameters=[{
                        'model_name', 'ssd_mobilenet_v2_coco',
                        'confidence_threshold', 0.7,
                        'input_tensor', 'input_tensor',
                        'input_format', 'nitros_tensor_list',
                    }],
                    remappings=[
                        ('/image_input', '/camera/color/image_rect_color'),
                    ]
                )
            ],
            output='screen'
        )
    ])
EOF

echo "Created perception pipeline launch file: $ISAAC_ROS_WS/src/isaac_ros_examples/launch/perception_pipeline.launch.py"

# Create documentation for Isaac ROS acceleration
cat << EOF > "$ISAAC_ROS_WS/src/isaac_ros_examples/doc/isaac_ros_acceleration_guide.md"
# Isaac ROS Acceleration Guide

## Overview
This guide explains how to use Isaac ROS packages for accelerated perception and navigation on humanoid robots. Isaac ROS packages leverage NVIDIA GPUs for accelerated processing of perception tasks like Visual SLAM, object detection, and tag detection.

## Prerequisites
- NVIDIA GPU with CUDA support
- Isaac Sim installed (see setup_isaac.sh)
- ROS 2 Humble
- TensorRT (for neural network acceleration)

## Installation
1. Run the setup script: \`./setup_isaac_ros_packages.sh\`
2. Source the workspace: \`source ~/isaac_ros_ws/install/setup.bash\`

## VSLAM (Visual SLAM)
The visual SLAM system provides pose estimation and map building capabilities:

### Launch
\`\`\`bash
ros2 launch isaac_ros_examples visual_slam.launch.py
\`\`\`

### Topics
- Input: \`/camera/left/image_rect_color\`, \`/camera/right/image_rect_color\`, \`/imu/data\`
- Output: \`/visual_slam/odometry\`, \`/visual_slam/map\`

## Perception Acceleration
Accelerated perception packages for object detection and landmark recognition:

### Launch
\`\`\`bash
ros2 launch isaac_ros_examples perception_pipeline.launch.py
\`\`\`

### Available Nodes
- **AprilTag Detection**: Detects AprilTag fiducial markers for precise localization
- **Object Detection**: Detects objects using neural networks

## Performance Tips
- Use appropriate image resolutions for your application (higher resolution = more compute)
- Adjust feature tracking parameters based on scene complexity
- Ensure proper lighting conditions for visual algorithms
- Calibrate cameras and IMU properly for best results

## Troubleshooting
- If CUDA errors occur, verify GPU drivers and CUDA toolkit installation
- If performance is poor, reduce image resolution or processing frequency
- Check camera calibration parameters if tracking is unstable

EOF

echo "Created Isaac ROS acceleration guide: $ISAAC_ROS_WS/src/isaac_ros_examples/doc/isaac_ros_acceleration_guide.md"

echo ""
echo "==============================================="
echo "ISAAC ROS ACCELERATION SETUP COMPLETED!"
echo "==============================================="
echo ""
echo "Setup includes:"
echo "- Isaac ROS acceleration configuration"
echo "- Scripts to install Isaac ROS packages for VSLAM and perception"
echo "- Example launch files for Visual SLAM and perception pipeline"
echo "- Documentation guide for Isaac ROS acceleration"
echo ""
echo "To complete the setup:"
echo "1. Run: $ISAAC_ROS_WS/setup_isaac_ros_packages.sh"
echo "2. Source the workspace: source $ISAAC_ROS_WS/install/setup.bash"
echo "3. See documentation: $ISAAC_ROS_WS/src/isaac_ros_examples/doc/isaac_ros_acceleration_guide.md"
echo ""
