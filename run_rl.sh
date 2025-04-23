#!/bin/bash

# Ensure we're using Python 3.8 for ROS 2 Foxy compatibility
export PYTHONPATH=/usr/lib/python3.8/site-packages:$PYTHONPATH

# Add local build directory to Python path using relative path
export PYTHONPATH="$(pwd)/build:$PYTHONPATH"

# Check for virtual environment and source it if it exists
VENV_PATH="./venv"
USING_VENV=false
if [ -d "$VENV_PATH" ]; then
    echo "Sourcing Python virtual environment from $VENV_PATH"
    source "$VENV_PATH/bin/activate"
    USING_VENV=true
    # Verify Python version in venv
    PYTHON_VERSION=$(python --version)
    echo "Using Python from virtual environment: $PYTHON_VERSION"
else
    echo "Virtual environment not found at $VENV_PATH"
    echo "Running with system Python. Consider creating a virtual environment with:"
    echo "python3.8 -m venv $VENV_PATH"
fi

# Source ROS2 Foxy environment
source /opt/ros/foxy/setup.bash

# Source the workspace environment first (if it exists)
if [ -f "ws_drone_comms/install/setup.bash" ]; then
    source ws_drone_comms/install/setup.bash
fi

# Use the appropriate Python interpreter
if [ "$USING_VENV" = true ]; then
    PYTHON_CMD="python"
else
    # Check for Python 3.8
    echo "Checking Python version..."
    if ! command -v python3.8 &> /dev/null; then
        echo "ERROR: Python 3.8 is required but not found on your system."
        echo "Please install Python 3.8 with: sudo apt install python3.8 python3.8-venv python3.8-dev"
        exit 1
    else
        echo "Python 3.8 found."
        PYTHON_CMD="python3.8"
    fi
fi

$PYTHON_CMD some_rl.py