# Quickstart Guide: Physical AI & Humanoid Robotics

## Prerequisites

### System Requirements
- **OS**: Ubuntu 20.04 LTS or newer (recommended) / macOS 12+ / Windows 10/11 (WSL2)
- **RAM**: 8GB minimum, 16GB recommended
- **Storage**: 5GB minimum for basic examples, 20GB for full simulation
- **GPU**: Optional but recommended for Isaac Sim (NVIDIA GPU with CUDA support)

### Software Dependencies
1. **ROS 2**: Humble Hawksbill (LTS) - Follow official installation guide
2. **Python**: 3.8 or newer
3. **Docker**: For containerized services (Qdrant, Postgres)
4. **Git**: Version control
5. **Node.js**: 16+ for Docusaurus documentation
6. **OpenAI API Key**: For AI integration examples

## Installation

### 1. Clone the Repository
```bash
git clone <repository-url>
cd physical-ai-humanoid-robotics
```

### 2. Set Up ROS 2 Environment
```bash
# Install ROS 2 Humble (Ubuntu)
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
```

### 3. Install Python Dependencies
```bash
# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install Python packages
pip install -r requirements.txt
```

### 4. Set Up Simulation Environment
```bash
# Install Gazebo Garden
sudo apt install ros-humble-gazebo-*
sudo apt install ignition-garden

# For NVIDIA Isaac Sim (optional)
# Follow NVIDIA's installation guide for Isaac Sim
```

### 5. Configure API Keys
```bash
# Create .env file in project root
cp .env.example .env
# Edit .env and add your OpenAI API key
```

### 6. Set Up RAG Services
```bash
# Using Docker Compose for Qdrant and Postgres
docker-compose up -d

# Or set up cloud services (Qdrant Cloud + Neon Postgres)
# Follow cloud provider setup instructions
```

## Running Your First Example

### 1. Build ROS Packages
```bash
cd ros_packages
colcon build --packages-select robot_control perception planning
source install/setup.bash
```

### 2. Run Basic Robot Simulation
```bash
# Launch robot in Gazebo simulation
ros2 launch simulation bringup.launch.py

# In another terminal, send a simple command
ros2 run robot_control move_basic
```

### 3. Test Voice Control
```bash
# Run the voice control node
ros2 run voice_control voice_node

# Speak commands like "move forward" or "stop"
```

### 4. Build and Serve Documentation
```bash
cd docs
npm install
npm run build
npm run serve
```

## Module-Specific Quickstarts

### Module 1: ROS 2 Nervous System
```bash
# Launch ROS 2 communication example
ros2 launch robot_control communication_demo.launch.py

# Monitor topics
ros2 topic list
ros2 topic echo /robot/joint_states
```

### Module 2: Digital Twin
```bash
# Launch simulation with robot and environment
ros2 launch simulation world_with_robot.launch.py

# View robot in Gazebo
# Check sensor data
ros2 topic echo /robot/camera/image_raw
```

### Module 3: AI-Robot Brain
```bash
# Launch planning and navigation
ros2 launch planning autonomous_navigation.launch.py

# Send navigation goals
ros2 run planning send_goal --x 1.0 --y 2.0
```

### Module 4: Vision-Language-Action
```bash
# Launch perception and AI integration
ros2 launch ai_bridge vla_pipeline.launch.py

# Test with sample commands
python3 examples/vla_example.py --command "pick up the red cube"
```

## Troubleshooting

### Common Issues

**ROS 2 Commands Not Found**
- Make sure you've sourced the ROS 2 environment: `source /opt/ros/humble/setup.bash`

**Gazebo Not Launching**
- Check if you have proper graphics support
- Try running with: `gazebo --verbose`

**Python Import Errors**
- Ensure you're using the virtual environment: `source venv/bin/activate`
- Reinstall dependencies: `pip install -r requirements.txt`

**Docker Services Not Starting**
- Check Docker is running: `sudo systemctl status docker`
- Check available disk space: `df -h`

### Getting Help
- Check the Troubleshooting section in each module
- Review the logs in `~/.ros/log/` for ROS 2 issues
- Use `ros2 doctor` to diagnose ROS 2 setup issues