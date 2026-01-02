# Multi-stage build for Physical AI & Humanoid Robotics development environment
ARG BASE=ubuntu:22.04

FROM ${BASE} as base

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Core utilities
    curl \
    wget \
    git \
    build-essential \
    cmake \
    pkg-config \
    unzip \
    # Python and development tools
    python3.8 \
    python3.8-dev \
    python3-pip \
    python3.8-venv \
    # Graphics and visualization
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    libgthread-2.0-0 \
    # ROS 2 dependencies
    software-properties-common \
    # OpenCV dependencies
    libopencv-dev \
    libeigen3-dev \
    # Boost for ORB-SLAM3
    libboost-dev \
    libboost-program-options-dev \
    libboost-system-dev \
    libboost-thread-dev \
    # Networking
    openssh-client \
    # Cleanup
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user
ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set up Python
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.8 1
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1

# Upgrade pip
RUN python3 -m pip install --upgrade pip

# Install ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    python3-argcomplete \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Switch to non-root user
USER $USERNAME
WORKDIR /home/$USERNAME

# Clone and build ORB_SLAM3 (simplified version for container)
RUN cd /home/$USERNAME \
    && git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git \
    && cd ORB_SLAM3 \
    && chmod +x build.sh \
    && ./build.sh

# Install Python packages
RUN python3 -m pip install --user \
    # Core scientific computing
    numpy \
    scipy \
    matplotlib \
    # ROS 2 Python
    rclpy \
    # Computer vision
    opencv-python \
    Pillow \
    # Machine learning
    torch \
    torchvision \
    scikit-learn \
    jupyter \
    # Robotics and simulation
    pybullet \
    transforms3d \
    pyquaternion \
    open3d \
    # Reinforcement learning
    stable-baselines3 \
    gymnasium \
    tensorboard \
    # Kinematics
    pin \
    ikpy \
    casadi \
    # VLA models
    transformers \
    open_clip_torch \
    accelerate \
    bitsandbytes \
    # Development tools
    pytest \
    pytest-cov \
    black \
    flake8 \
    # Jupyter notebook support
    notebook \
    ipykernel

# Set up ROS 2 environment in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

# Create workspace for examples
RUN mkdir -p /home/$USERNAME/workspace/src \
    && cd /home/$USERNAME/workspace \
    && source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install

# Copy project files
COPY --chown=$USERNAME:$USER_UID . /home/$USERNAME/workspace/src/humanoid_robotic_book

# Set working directory
WORKDIR /home/$USERNAME/workspace

# Expose ports for Jupyter and web services
EXPOSE 8888 8080

# Entry point
COPY docker-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker-entrypoint.sh
ENTRYPOINT ["docker-entrypoint.sh"]
CMD ["bash"]