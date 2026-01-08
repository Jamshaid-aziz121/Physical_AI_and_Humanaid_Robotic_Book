#!/bin/bash

# Setup script for NVIDIA Isaac Sim for the Physical AI & Humanoid Robotics Book
# This script installs and configures Isaac Sim for synthetic data generation and perception

set -e  # Exit on any error

echo "========================================="
echo "Setting up NVIDIA Isaac Sim for Robotics"
echo "========================================="

# Check if running on a supported platform
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    PLATFORM="linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    echo "Error: This script currently supports Linux only"
    exit 1
else
    echo "Error: Unsupported platform: $OSTYPE"
    exit 1
fi

# Check for NVIDIA GPU and driver
echo "Checking NVIDIA GPU and driver..."
if ! command -v nvidia-smi &> /dev/null; then
    echo "Error: nvidia-smi not found. Please install NVIDIA drivers."
    exit 1
fi

GPU_INFO=$(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader,nounits)
echo "Detected GPU: $GPU_INFO"

# Check CUDA version
CUDA_VERSION=$(nvcc --version 2>/dev/null | grep -oP 'V\d+\.\d+' | cut -c2-)
if [ -z "$CUDA_VERSION" ]; then
    CUDA_VERSION=$(nvidia-smi | grep -oP 'CUDA Version: \K\d+\.\d+')
fi

echo "Detected CUDA version: $CUDA_VERSION"

# Check if CUDA 11.x or 12.x is available (Isaac Sim requirement)
CUDA_MAJOR=$(echo $CUDA_VERSION | cut -d. -f1)
if [ "$CUDA_MAJOR" -lt 11 ]; then
    echo "Error: CUDA 11.x or higher is required. Detected: $CUDA_VERSION"
    exit 1
fi

echo "CUDA version check passed"

# Create Isaac directory structure
ISAAC_ROOT="$HOME/isaac_sim"
mkdir -p "$ISAAC_ROOT"
cd "$ISAAC_ROOT"

echo "Created Isaac Sim directory: $ISAAC_ROOT"

# Check if Isaac Sim is already installed
if [ -d "isaac-sim" ]; then
    echo "Isaac Sim appears to be already installed in $ISAAC_ROOT/isaac-sim"
    read -p "Do you want to reinstall? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf isaac-sim
    else
        echo "Skipping installation"
        exit 0
    fi
fi

# Download Isaac Sim (using the public download link format)
echo "Downloading Isaac Sim..."

# For the book, we'll provide instructions for downloading Isaac Sim
# In practice, users would need to download from NVIDIA Developer portal
cat << EOF > download_instructions.txt
==================================================
ISAAC SIM DOWNLOAD INSTRUCTIONS
==================================================

To download Isaac Sim:

1. Visit: https://developer.nvidia.com/isaac-sim
2. Create or login to your NVIDIA Developer account
3. Download Isaac Sim for Linux (latest version)
4. Place the downloaded archive in this directory: $ISAAC_ROOT

Then run this script again, or follow the manual installation steps below:

Manual Installation:
1. Extract the downloaded archive: tar -xzf isaac-sim-[version].tar.gz
2. Rename the extracted folder to 'isaac-sim'
3. Run: python -s isaac-sim/setup_python_env.sh
4. Follow the post-installation steps below

EOF

echo "Download instructions saved to: $ISAAC_ROOT/download_instructions.txt"

# Check if Isaac Sim directory exists after download attempt
if [ -d "isaac-sim" ]; then
    echo "Installing Isaac Sim dependencies..."

    # Setup Python environment for Isaac Sim
    cd isaac-sim
    if [ -f "setup_python_env.sh" ]; then
        ./setup_python_env.sh
    else
        echo "Warning: setup_python_env.sh not found. Using manual setup."
    fi

    # Create a convenience script to launch Isaac Sim
    cat << 'EOF' > launch_isaac_sim.sh
#!/bin/bash
# Convenience script to launch Isaac Sim

ISAAC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_PATH="$ISAAC_ROOT/isaac-sim"

if [ ! -d "$ISAAC_PATH" ]; then
    echo "Error: Isaac Sim not found at $ISAAC_PATH"
    exit 1
fi

cd "$ISAAC_PATH"

# Activate Isaac Sim environment
source setup_python_env.sh 2>/dev/null || true

# Launch Isaac Sim
./python.sh -m omni.isaac.kit --exec-path omni.isaac.examples.simple_robots.carter_franka_pick_place

EOF

    chmod +x launch_isaac_sim.sh
    echo "Created launch script: $ISAAC_ROOT/isaac-sim/launch_isaac_sim.sh"

    # Create Isaac Sim configuration file
    cat << EOF > isaac_sim_config.yaml
# Isaac Sim Configuration for Physical AI & Humanoid Robotics Book

isaac_sim:
  version: "4.2.0"  # Or whatever version is installed
  path: "$ISAAC_ROOT/isaac-sim"
  assets_path: "$ISAAC_ROOT/isaac-sim/assets"

  # Synthetic data generation settings
  synthetic_data:
    enable: true
    camera:
      resolution: [1280, 720]
      fov: 60  # degrees
    lidar:
      enable: true
      rotation_frequency: 10  # Hz
      points_per_second: 100000

  # Simulation settings
  simulation:
    physics_frequency: 60  # Hz
    rendering_frequency: 60  # Hz
    max_step_size: 0.0167  # ~60 FPS

  # Robot settings
  robot:
    default_robot: "carter_franka"
    joint_limits:
      enable: true
    collision_avoidance:
      enable: true
      threshold: 0.1  # meters

EOF

    echo "Created Isaac Sim configuration: $ISAAC_ROOT/isaac-sim/isaac_sim_config.yaml"

    # Create Isaac ROS bridge setup
    cat << 'EOF' > setup_isaac_ros_bridge.sh
#!/bin/bash
# Setup script for Isaac ROS Bridge

# Install Isaac ROS packages if not already installed
if [ ! -d "$HOME/isaac_ros_ws/src" ]; then
    mkdir -p $HOME/isaac_ros_ws/src
    cd $HOME/isaac_ros_ws

    # Initialize workspace
    colcon build --symlink-install
fi

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Clone Isaac ROS bridge packages (example packages)
cd $HOME/isaac_ros_ws/src

# For the book, we'll list the common Isaac ROS packages
# Users would need to clone these from the Isaac ROS repositories
cat << INNER_EOF > isaac_ros_packages_list.txt
Common Isaac ROS Packages for the book:
- isaac_ros_apriltag
- isaac_ros_ball_detection
- isaac_ros_bezier_planner
- isaac_ros_common
- isaac_ros_detectnet
- isaac_ros_freespace_segmentation
- isaac_ros_hawks
- isaac_ros_image_pipeline
- isaac_ros_managed_nitros
- isaac_ros_messages
- isaac_ros_nitros
- isaac_ros_point_cloud_interfaces
- isaac_ros_realsense
- isaac_ros_se3_estimator
- isaac_ros_vda5050
- isaac_ros_visual_slam
- isaac_ros_yaml_param_parser
INNER_EOF

echo "Isaac ROS packages list saved to: $HOME/isaac_ros_ws/src/isaac_ros_packages_list.txt"

EOF

    chmod +x setup_isaac_ros_bridge.sh
    echo "Created Isaac ROS Bridge setup script"

    echo ""
    echo "========================================="
    echo "ISAAC SIM SETUP COMPLETED SUCCESSFULLY!"
    echo "========================================="
    echo ""
    echo "Next steps:"
    echo "1. Ensure Isaac Sim is properly installed in $ISAAC_ROOT/isaac-sim"
    echo "2. Run: $ISAAC_ROOT/isaac-sim/launch_isaac_sim.sh to launch Isaac Sim"
    echo "3. Run: $ISAAC_ROOT/isaac-sim/setup_isaac_ros_bridge.sh to setup ROS bridge"
    echo "4. See the configuration file for synthetic data generation settings"
    echo ""
    echo "For more information, visit: https://docs.omniverse.nvidia.com/isaacsim/latest/"
else
    echo ""
    echo "========================================="
    echo "ISAAC SIM SETUP PARTIALLY COMPLETED"
    echo "========================================="
    echo ""
    echo "Isaac Sim was not found in $ISAAC_ROOT/isaac-sim"
    echo "Please follow the download instructions in: $ISAAC_ROOT/download_instructions.txt"
    echo ""
    echo "Once downloaded, re-run this script or follow the manual installation steps."
fi

echo ""
echo "Setup completed. Check $ISAAC_ROOT for installation files."