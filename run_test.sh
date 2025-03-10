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

# Parse command line arguments
TEST_TYPE="basic"  # Changed default from "takeoff" to "basic" to match test_drone_env.py
if [ "$1" == "--test" ]; then
    TEST_TYPE="$2"
fi

# Check if PX4 simulation is running by looking for required topics
echo "Checking if PX4 simulation is running..."
VEHICLE_ODOM_TOPIC=$(ros2 topic list | grep "/fmu/out/vehicle_odometry")
if [ -z "$VEHICLE_ODOM_TOPIC" ]; then
    echo "WARNING: PX4 simulation odometry topic not found!"
else
    echo "Found odometry topic: $VEHICLE_ODOM_TOPIC"
fi

# Check topic structure
echo "Checking available drone topics:"
ros2 topic list | grep "/fmu/" | sort

# Check message type for odometry topic
echo "Checking message type for odometry topic..."
ros2 topic info /fmu/out/vehicle_odometry

# Echo sample odometry message
echo "Trying to read odometry data (will wait for 5 seconds)..."
timeout 5s ros2 topic echo /fmu/out/vehicle_odometry | head -n 20

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

# Run diagnostic script to check PX4 messages with relative path
echo "Running PX4 message diagnostics..."
$PYTHON_CMD ./check_px4_messages.py

# Run the test script
echo "Running test: $TEST_TYPE"
$PYTHON_CMD test_drone_env.py --test $TEST_TYPE