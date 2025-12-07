# Development Environment Setup

This guide covers setting up a complete development environment for working with the Physical AI & Humanoid Robotics textbook examples.

## Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04 LTS (primary), Windows 10/11, macOS 13+ (secondary)
- **CPU**: 8+ cores recommended
- **RAM**: 16GB minimum, 32GB recommended (for Isaac Sim)
- **GPU**: NVIDIA RTX 3060 or better (required for Isaac Sim)
- **Storage**: 50GB free space for Isaac Sim and assets

### Software Dependencies

#### Core Dependencies
```bash
# Python 3.8+ (Ubuntu/Debian)
sudo apt update
sudo apt install python3.8 python3.8-dev python3-pip
sudo apt install python3.8-venv

# Build tools
sudo apt install build-essential cmake pkg-config

# Git
sudo apt install git
```

#### ROS 2 Humble (Ubuntu)
```bash
# Add ROS 2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-argcomplete
```

## Project Setup

### 1. Clone Repository
```bash
git clone https://github.com/your-org/humanoid-robotic-book.git
cd humanoid-robotic-book
```

### 2. Create Python Virtual Environment
```bash
python3.8 -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
```

### 3. Install Python Dependencies
```bash
# Core robotics libraries
pip install numpy scipy matplotlib
pip install rclpy  # ROS 2 Python client

# Computer vision
pip install opencv-python
pip install Pillow

# Machine learning
pip install torch torchvision
pip install scikit-learn
pip install jupyter

# 3D simulation and robotics
pip install pybullet
pip install transforms3d
pip install pyquaternion

# SLAM and navigation
pip install open3d
pip install evo
```

## Chapter-Specific Setups

### Chapter 8: NVIDIA Isaac Sim

1. **Download Isaac Sim 2024.1.0**
   - Visit [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)
   - Download the appropriate version for your OS
   - Requires NVIDIA account and GPU

2. **Installation (Linux)**
```bash
# Extract Isaac Sim
tar -xzf Isaac-Sim_2024.1.0.tar.gz
cd isaac-sim

# Run setup script
./python.sh setup.py
```

3. **ROS 2 Bridge Setup**
```bash
# Install Isaac Sim ROS 2 extension
./python.sh -m pip install isaac-ros2-bridge
```

### Chapter 9: Visual SLAM & Navigation

1. **ORB-SLAM3 Installation**
```bash
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3

# Install dependencies
sudo apt install libeigen3-dev
sudo apt install libopencv-dev
sudo apt install libboost-dev

# Build
chmod +x build.sh
./build.sh
```

2. **Nav2 Dependencies**
```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
```

### Chapter 10: Reinforcement Learning

1. **Stable-Baselines3**
```bash
pip install stable-baselines3[extra]
pip install gymnasium
pip install tensorboard
```

2. **Isaac Gym (Optional, for GPU acceleration)**
   - Download from NVIDIA Developer Portal
   - Follow installation guide in documentation

### Chapter 11: Humanoid Kinematics

1. **Pinocchio (Inverse Kinematics)**
```bash
# Ubuntu
sudo apt install libpinocchio-dev

# Or pip install
pip install pin
```

2. **Additional Kinematics Libraries**
```bash
pip install ikpy
pipCASADI
```

### Chapter 12: VLA & Conversational Robotics

1. **OpenCLIP**
```bash
pip install open_clip_torch
```

2. **BLIP-2 Dependencies**
```bash
pip install transformers
pip install accelerate
pip install bitsandbytes
```

## IDE Configuration

### VS Code Extensions
```json
{
  "recommendations": [
    "ms-python.python",
    "ms-iot.vscode-ros",
    "ms-python.black-formatter",
    "ms-python.flake8",
    "ms-azuretools.vscode-docker",
    "ms-vscode.cpptools",
    "redhat.vscode-yaml"
  ]
}
```

### VS Code Settings
```json
{
  "python.defaultInterpreterPath": "${workspaceFolder}/.venv/bin/python",
  "python.linting.enabled": true,
  "python.linting.flake8Enabled": true,
  "python.formatting.provider": "black",
  "editor.formatOnSave": true,
  "files.associations": {
    "*.urdf": "xml",
    "*.xacro": "xml",
    "*.sdf": "xml"
  }
}
```

## Testing Setup

### Unit Tests
```bash
# Install test dependencies
pip install pytest pytest-cov

# Run tests
pytest tests/ --cov=examples
```

### ROS 2 Tests
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash
source install/setup.bash  # If using colcon

# Run ROS 2 tests
colcon test --packages-select your_package
```

## Docker Setup (Optional)

For consistent development environment:

```bash
# Build Docker image
docker build -t humanoid-robotics-dev .

# Run container
docker run -it --gpus all \
  -v $(pwd):/workspace \
  --network host \
  humanoid-robotics-dev
```

## Troubleshooting

### Common Issues

1. **Isaac Sim GPU Issues**
   - Ensure NVIDIA drivers are up to date
   - Check CUDA version compatibility
   - Verify GPU is recognized: `nvidia-smi`

2. **ROS 2 Build Failures**
   - Clear build cache: `rm -rf build/ install/ log/`
   - Re-source workspace after builds

3. **Python Import Errors**
   - Ensure virtual environment is activated
   - Check pip version: `pip --version`
   - Reinstall problematic packages

### Performance Tips

1. **Isaac Sim Optimization**
   - Use DLSS for improved performance
   - Adjust physics settings for your use case
   - Use scene optimization tools

2. **SLAM Performance**
   - Adjust image resolution for real-time performance
   - Use GPU acceleration when available
   - Tune feature extraction parameters

## Contributing

### Code Style
- Python: Follow PEP 8, use Black formatter
- C++: Follow Google Style Guide
- ROS 2: Follow ROS 2 conventions

### Testing
- Write unit tests for new functionality
- Test on multiple platforms when possible
- Verify examples work with latest dependencies

### Documentation
- Update README for new examples
- Add docstrings to all functions
- Include system requirements in examples