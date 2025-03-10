#!/usr/bin/env python3.8
"""
Diagnostic script to check PX4 message formats and topic availability
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import time
import sys
import os

# Add the path to the PX4 messages
ws_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ws_drone_comms")
sys.path.append(os.path.join(ws_path, "install/px4_msgs/lib/python3.8/site-packages"))
sys.path.append(os.path.join(ws_path, "build/px4_msgs"))
sys.path.append(os.path.join(ws_path, "src/px4_msgs"))

# Also add local install directory to ensure we find the right modules
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "install/px4_msgs/lib/python3.8/site-packages"))

from px4_msgs.msg import (
    VehicleOdometry, 
    VehicleStatus,
    VehicleAttitude,
    VehicleLocalPosition,
    VehicleCommand,
    OffboardControlMode,
    VehicleAttitudeSetpoint
)

class PX4MessageChecker(Node):
    def __init__(self):
        super().__init__('px4_message_checker')
        self.get_logger().info('PX4 Message Checker initialized')
        
        # Create a timer for diagnostics - 2 seconds to reduce verbosity
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        # Track received message types
        self.received_msgs = {}
        
        # Create QoS profile for PX4 messages - This is the key change
        # PX4 uses default QoS with best effort reliability and transient local durability
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # List of topics to check with their corresponding message types
        topics = {
            '/fmu/out/vehicle_odometry': VehicleOdometry,
            '/fmu/out/vehicle_status': VehicleStatus,
            '/fmu/out/vehicle_attitude': VehicleAttitude,
            '/fmu/out/vehicle_local_position': VehicleLocalPosition,
            '/fmu/in/vehicle_command': VehicleCommand,
            '/fmu/in/offboard_control_mode': OffboardControlMode,
            '/fmu/in/vehicle_attitude_setpoint': VehicleAttitudeSetpoint
        }
        
        # Create subscriptions with the PX4 QoS profile
        self.topic_subscriptions = []
        for topic, msg_type in topics.items():
            self.topic_subscriptions.append(
                self.create_subscription(
                    msg_type, 
                    topic, 
                    lambda msg, t=topic: self.generic_callback(msg, t),
                    qos_profile
                )
            )
        
        self.get_logger().info('Subscriptions created. Waiting for messages...')
    
    def generic_callback(self, msg, topic):
        if topic not in self.received_msgs:
            self.get_logger().info(f'First message received on {topic}')
            self.received_msgs[topic] = {
                'count': 1,
                'type': str(type(msg)),
                'sample': str(msg)[:200] + '...' if len(str(msg)) > 200 else str(msg)  # Shorter sample
            }
        else:
            self.received_msgs[topic]['count'] += 1
    
    def timer_callback(self):
        # Reduced verbosity - only log overall count
        if self.received_msgs:
            topic_counts = [f"{topic.split('/')[-1]}: {data['count']}" for topic, data in self.received_msgs.items()]
            self.get_logger().info(f"Messages received: {', '.join(topic_counts)}")
        else:
            self.get_logger().warn('No messages received from any PX4 topics yet')

def main():
    rclpy.init()
    node = PX4MessageChecker()
    
    # Time to collect data
    print("Checking PX4 messages. Will collect data for 10 seconds...")
    print("Make sure the PX4 SITL simulation is running in Gazebo")
    
    try:
        start_time = time.time()
        while (time.time() - start_time) < 10.0:
            rclpy.spin_once(node, timeout_sec=0.1)

            # Print status every 2 seconds
            last_print_time = -1
            elapsed_seconds = int(time.time() - start_time)
            if elapsed_seconds % 2 == 0 and elapsed_seconds > 0 and elapsed_seconds != last_print_time:
                if elapsed_seconds % 16 == 0:  # Even sparser output
                    print(f"Collecting data... ({elapsed_seconds}s)")
                    last_print_time = elapsed_seconds
    except KeyboardInterrupt:
        print("Interrupted by user")
    
    # Print summary
    print("\n=== PX4 Message Summary ===")
    if node.received_msgs:
        # Just print counts for conciseness
        print("Topics with messages received:")
        for topic, data in node.received_msgs.items():
            print(f"- {topic}: {data['count']} messages, type: {data['type'].split('.')[-1]}")
    else:
        print("No messages received. Ensure PX4 simulation is running.")
        print("\nCheck if the simulation is running with:")
        print("  ros2 topic list | grep fmu")
        print("  ros2 topic echo /fmu/out/vehicle_odometry --once")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()