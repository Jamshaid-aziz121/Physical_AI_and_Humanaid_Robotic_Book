# Quickstart Guide for Physical AI & Humanoid Robotics Book

## Prerequisites

Before starting with the Physical AI & Humanoid Robotics Book, ensure you have the following prerequisites installed:

### System Requirements
- Ubuntu 22.04 LTS (recommended) or compatible Linux distribution
- At least 8GB RAM (16GB recommended)
- At least 50GB free disk space
- Dedicated GPU with CUDA support (for NVIDIA Isaac features)

### Software Requirements
1. **ROS 2 Humble Hawksbill** (or Iron Irwini)
   ```bash
   sudo apt update
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

2. **Python 3.10+**
   ```bash
   python3 --version
   # If not installed: sudo apt install python3.10
   ```

3. **Node.js 16+ and npm** (for Docusaurus documentation)
   ```bash
   node --version
   npm --version
   # If not installed: sudo apt install nodejs npm
   ```

4. **Gazebo Garden** (for simulation)
   ```bash
   sudo apt install ros-humble-gazebo-*
   ```

5. **Git** for version control
   ```bash
   git --version
   # If not installed: sudo apt install git
   ```

## Setting Up the Development Environment

1. **Install ROS 2 dependencies:**
   ```bash
   sudo apt update
   sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
   sudo rosdep init
   rosdep update
   ```

2. **Source ROS 2 environment:**
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Clone the book repository:**
   ```bash
   mkdir -p ~/physical_ai_book/src
   cd ~/physical_ai_book
   git clone <repository-url> src/
   ```

4. **Install Python dependencies:**
   ```bash
   pip3 install -r src/requirements.txt
   ```

## Running Your First Simulation

1. **Navigate to the ROS 2 workspace:**
   ```bash
   cd ~/physical_ai_book
   source /opt/ros/humble/setup.bash
   colcon build
   source install/setup.bash
   ```

2. **Launch the basic humanoid robot simulation:**
   ```bash
   ros2 launch book_examples basic_humanoid_simulation.launch.py
   ```

3. **In another terminal, send commands to the robot:**
   ```bash
   ros2 topic pub /joint_commands sensor_msgs/msg/JointState '{name: ["joint1", "joint2"], position: [0.5, -0.3]}'
   ```

## Exploring the Book Modules

### Module 1: ROS 2 Fundamentals
1. Navigate to the ROS 2 examples:
   ```bash
   cd ~/physical_ai_book/src/modules/01-ros2-fundamentals/code-examples
   ```

2. Build and run the publisher/subscriber example:
   ```bash
   cd publisher_subscriber_example
   colcon build
   source install/setup.bash
   ros2 run examples_py talker
   # In another terminal: ros2 run examples_py listener
   ```

### Module 2: Digital Twin Simulation
1. Launch Gazebo with a humanoid robot:
   ```bash
   ros2 launch book_examples simple_humanoid_gazebo.launch.py
   ```

2. Visualize in RViz:
   ```bash
   ros2 run rviz2 rviz2 -d ~/physical_ai_book/src/config/humanoid.rviz
   ```

### Module 3: AI Navigation & Perception
1. Start the Isaac navigation example:
   ```bash
   ros2 launch nav_isaac_examples turtlebot3_navigation2.launch.py
   ```

2. Send navigation goals:
   ```bash
   ros2 run nav2_msgs navigation_goal_publisher
   ```

### Module 4: Vision-Language-Action Integration
1. Set up your LLM API credentials (create `.env` file):
   ```
   LLM_API_KEY=your_api_key_here
   LLM_BASE_URL=https://api.provider.com/v1
   ```

2. Run the voice command processor:
   ```bash
   ros2 run vla_examples voice_to_action_node
   ```

## Building the Documentation

1. **Navigate to the documentation directory:**
   ```bash
   cd ~/physical_ai_book/website/docusaurus
   ```

2. **Install dependencies:**
   ```bash
   npm install
   ```

3. **Start the documentation server:**
   ```bash
   npm start
   ```

4. **Access the documentation in your browser:**
   - Open `http://localhost:3000`

## Validation Commands

Run these commands to validate your setup:

1. **Verify ROS 2 installation:**
   ```bash
   ros2 topic list
   ros2 node list
   ```

2. **Test basic ROS 2 functionality:**
   ```bash
   ros2 run demo_nodes_cpp talker &
   ros2 run demo_nodes_py listener
   ```

3. **Check Gazebo availability:**
   ```bash
   gazebo --version
   ```

4. **Validate simulation:**
   ```bash
   ros2 launch book_examples validation_test.launch.py
   ```

## Troubleshooting

**Issue**: Cannot find ROS 2 packages
- Solution: Ensure you've sourced the setup.bash file and built the workspace with `colcon build`

**Issue**: Gazebo fails to start
- Solution: Check if you have proper graphics drivers installed and X11 forwarding if using SSH

**Issue**: LLM examples fail with authentication errors
- Solution: Verify your API key is correctly set in the environment variables

## Next Steps

After completing the setup:

1. Proceed to Module 1: ROS 2 Fundamentals
2. Work through the exercises in sequence
3. Experiment with the code examples
4. Attempt the module-specific projects
5. Move to the next module when comfortable with the concepts