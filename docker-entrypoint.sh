#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f "/home/rosdev/workspace/install/setup.bash" ]; then
    source /home/rosdev/workspace/install/setup.bash
fi

# Add workspace to Python path
export PYTHONPATH="/home/rosdev/workspace/src:$PYTHONPATH"
export PYTHONPATH="/home/rosdev/.local/lib/python3.8/site-packages:$PYTHONPATH"

# Set display for GUI applications
export DISPLAY=${DISPLAY:-:0}

# Function to start Jupyter notebook
start_jupyter() {
    echo "Starting Jupyter notebook..."
    cd /home/rosdev
    jupyter notebook --ip=0.0.0.0 --port=8888 --no-browser --allow-root --NotebookApp.token='' --NotebookApp.password=''
}

# Function to build workspace
build_workspace() {
    echo "Building ROS 2 workspace..."
    cd /home/rosdev/workspace
    colcon build --symlink-install
}

# Function to run tests
run_tests() {
    echo "Running tests..."
    cd /home/rosdev/workspace
    colcon test
    colcon test-result --verbose
}

# Check for command line arguments
if [ "$1" = "jupyter" ]; then
    start_jupyter
elif [ "$1" = "build" ]; then
    build_workspace
elif [ "$1" = "test" ]; then
    run_tests
else
    # Default: start bash
    exec "$@"
fi