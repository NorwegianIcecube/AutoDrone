# AutoDrone
Train a drone in simulated environment such that it autonomously navigates to the target pick location, pick it from that location and deliver to target goal location.

## Table of Contents
1. [Introduction](#introduction)
2. [Installation](#installation)
3. [Usage](#usage)
4. [Results](#results)
5. [Future Work](#future-work)
6. [References](#references)

## Introduction
This project implements an autonomous drone navigation system using PX4 and ROS2 in a simulated environment.

## Installation

### Prerequisites
- Ubuntu 20.04
- Python 3.8 (specific version required)
- ROS2 Foxy
- PX4 Autopilot
- Gazebo Classic

### Setting up the environment

1. Clone the repository:
```bash
git clone https://github.com/NorwegianIcecube/AutoDrone.git
cd AutoDrone
```

2. Create and activate a Python virtual environment:
```bash
python3.8 -m venv venv
source venv/bin/activate
```

3. Install the required Python packages:
```bash
pip install -r requirements.txt
```

4. Install ROS dependencies using rosdep:
```bash
sudo apt update
sudo apt install -y python3-rosdep
sudo rosdep init # Skip if already initialized
rosdep update
rosdep install --from-paths . --ignore-src -y
```

5. Build the ROS2 workspace:
```bash
colcon build
source install/setup.bash
```

## Usage

### Starting the Simulation

1. Source ROS2 Foxy setup:
```bash
source /opt/ros/foxy/setup.bash
```

2. Start the DDS agent (in a new terminal):
```bash
source /opt/ros/foxy/setup.bash
micro-xrce-dds-agent udp4 -p 8888
```

3. Launch PX4 SITL simulation with Gazebo (in another terminal):
```bash
source /opt/ros/foxy/setup.bash
cd /path/to/PX4-Autopilot
make px4_sitl gazebo-classic
```

4. In a new terminal, source all required environments and run the test script:
```bash
source /opt/ros/foxy/setup.bash
source venv/bin/activate
source install/setup.bash
./run_test.sh
```

## Results
TBD

## Future Work
TBD

## References
- [PX4 Documentation](https://docs.px4.io/)
- [ROS2 Documentation](https://docs.ros.org/en/foxy/)