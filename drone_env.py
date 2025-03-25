#!/usr/bin/env python3
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import time
import threading
import sys
import os
import logging

# Import PX4 messages
try:
    from px4_msgs.msg import (
        VehicleOdometry, 
        VehicleCommand, 
        OffboardControlMode, 
        TrajectorySetpoint, 
        VehicleStatus
    )
except ImportError:
    print("ERROR: Could not import px4_msgs. Make sure you've sourced ROS2 environment.")
    sys.exit(1)


class PX4DroneEnv(gym.Env):
    """
    Simplified PX4 drone environment for reinforcement learning
    Focuses on core functionality: odometry, offboard control, arming, and basic control inputs
    """
    def __init__(self, debug=False):
        """Initialize the drone environment
        
        Args:
            debug: Enable debug logging
        """
        # Set up logging
        self.logger = logging.getLogger('px4_drone_env')
        if debug:
            logging.basicConfig(level=logging.DEBUG)
        else:
            logging.basicConfig(level=logging.INFO)
        
        self.logger.info('Simplified Drone Environment initialized')
        
        # Initialize ROS2
        rclpy.init(args=None)
        self.node = rclpy.create_node('px4_drone_env')
        
        # Create an executor and spin it in a separate thread
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self._spin_thread, daemon=True)
        self.spin_thread.start()
        
        # Create publishers and subscribers
        self._create_publishers()
        self._create_subscribers()
        
        # Stored state
        self.current_state = np.zeros(12)  # x, y, z, vx, vy, vz, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate
        self.received_odometry = False
        self.armed = False
        self.offboard_mode = False
        self.last_offboard_timestamp = 0
        self.timestamp = 0
        
        # Define action and observation spaces
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0, -1.0, 0.0]),  # roll, pitch, yaw_rate, thrust
            high=np.array([1.0, 1.0, 1.0, 1.0]),
            dtype=np.float32
        )
        
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(12,),  # State dimensions
            dtype=np.float32
        )
        
        # Wait for odometry data
        self._wait_for_odometry()
        
    def _create_publishers(self):
        """Set up ROS2 publishers"""
        # Quality of Service settings
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers for drone control
        self.offboard_control_mode_publisher = self.node.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        
        self.trajectory_setpoint_publisher = self.node.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        self.vehicle_command_publisher = self.node.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
    
    def _create_subscribers(self):
        """Set up ROS2 subscribers"""
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Odometry subscriber
        self.odometry_sub = self.node.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self._odometry_callback,
            qos_profile
        )
        
        # Vehicle status subscriber
        self.vehicle_status_sub = self.node.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self._vehicle_status_callback,
            qos_profile
        )
    
    def _spin_thread(self):
        """Thread function to spin the ROS2 node"""
        self.executor.spin()
        
    def _wait_for_odometry(self, timeout=10):
        """Wait for initial odometry data"""
        print(f"Waiting for odometry data (timeout: {timeout} seconds)...")
        start_time = time.time()
        while not self.received_odometry and time.time() - start_time < timeout:
            # Only print every 3 seconds to reduce verbosity
            if int(time.time() - start_time) % 3 == 0 and int(time.time() - start_time) > 0:
                print(f"Waiting... (elapsed: {int(time.time() - start_time)}s)")
            time.sleep(1.0)
        
        if not self.received_odometry:
            print("WARNING: Timed out waiting for odometry data")
        else:
            print(f"Received odometry data after {time.time() - start_time:.1f} seconds")
            print(f"Initial position: x={self.current_state[0]:.2f}, y={self.current_state[1]:.2f}, z={self.current_state[2]:.2f}")
    
    def _odometry_callback(self, msg):
        """Process the odometry data from the drone"""
        if not self.received_odometry:
            self.received_odometry = True
            print("First odometry message received!")
            
        # Extract position
        self.current_state[0] = msg.position[0]  # x
        self.current_state[1] = msg.position[1]  # y
        self.current_state[2] = msg.position[2]  # z
        
        # Extract velocity
        self.current_state[3] = msg.velocity[0]  # vx
        self.current_state[4] = msg.velocity[1]  # vy
        self.current_state[5] = msg.velocity[2]  # vz
        
        # Convert quaternion to Euler angles
        q = msg.q
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
        cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        self.current_state[6] = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        if abs(sinp) >= 1:
            self.current_state[7] = np.copysign(np.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            self.current_state[7] = np.arcsin(sinp)
            
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        self.current_state[8] = np.arctan2(siny_cosp, cosy_cosp)
        
        # Extract angular velocity
        self.current_state[9] = msg.angular_velocity[0]   # roll rate
        self.current_state[10] = msg.angular_velocity[1]  # pitch rate
        self.current_state[11] = msg.angular_velocity[2]  # yaw rate
        
        self.timestamp = msg.timestamp
    
    def _vehicle_status_callback(self, msg):
        """Monitor vehicle status changes"""
        # Track arming state changes
        self.armed = (msg.arming_state == 2)  # 2 = ARMED
        
        # Track nav state for offboard mode
        self.offboard_mode = (msg.nav_state == 14)  # 14 = OFFBOARD mode
        
        # Log state changes for debugging
        if hasattr(self, '_prev_arming_state') and msg.arming_state != self._prev_arming_state:
            arm_state_names = {1: "DISARMED", 2: "ARMED"}
            state_name = arm_state_names.get(msg.arming_state, f"UNKNOWN({msg.arming_state})")
            self.logger.info(f"Arming state changed: {state_name}")
        self._prev_arming_state = msg.arming_state
        
        if hasattr(self, '_prev_nav_state') and msg.nav_state != self._prev_nav_state:
            nav_state_names = {
                0: "MANUAL", 14: "OFFBOARD",
                # Add more states as needed
            }
            state_name = nav_state_names.get(msg.nav_state, f"OTHER({msg.nav_state})")
            self.logger.info(f"Navigation state changed: {state_name}")
        self._prev_nav_state = msg.nav_state
    
    def _publish_offboard_control_mode(self):
        """Publish the offboard control mode message"""
        msg = OffboardControlMode()
        msg.timestamp = int(time.time() * 1e6)  # Convert to microseconds
        # We'll use attitude + thrust control mode (direct roll, pitch, yaw rate, thrust)
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = True
        msg.body_rate = False
        msg.thrust_and_torque = False
        
        self.offboard_control_mode_publisher.publish(msg)
        self.last_offboard_timestamp = time.time()
    
    def _publish_attitude_setpoint(self, roll, pitch, yaw_rate, thrust):
        """Publish the attitude setpoint for direct roll, pitch, yaw rate, thrust control"""
        from px4_msgs.msg import VehicleAttitudeSetpoint
        
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = int(time.time() * 1e6)
        
        # Convert roll/pitch from -1,1 range to radians (max ±0.5 rad, ~30 degrees)
        roll_rad = roll * 0.5
        pitch_rad = pitch * 0.5
        
        # Convert to quaternion
        cy = np.cos(0 * 0.5)  # We use 0 for yaw since we're controlling yaw rate
        sy = np.sin(0 * 0.5)
        cp = np.cos(pitch_rad * 0.5)
        sp = np.sin(pitch_rad * 0.5)
        cr = np.cos(roll_rad * 0.5)
        sr = np.sin(roll_rad * 0.5)
        
        q0 = cy * cp * cr + sy * sp * sr
        q1 = cy * cp * sr - sy * sp * cr
        q2 = cy * sp * cr + sy * cp * sr
        q3 = sy * cp * cr - cy * sp * sr
        
        msg.q_d = [float(q0), float(q1), float(q2), float(q3)]
        
        # Set yaw rate (scaled from [-1,1] to [-1,1] rad/s)
        msg.yaw_sp_move_rate = float(yaw_rate)
        
        # Set thrust (scaled from [0,1] to [0,1])
        msg.thrust_body = [0.0, 0.0, -float(thrust)]  # Negative z is up in body frame
        
        # Publish the message through the corresponding publisher
        # Need to create this publisher in _create_publishers if not already present
        if not hasattr(self, 'attitude_setpoint_publisher'):
            qos_profile = rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=10
            )
            self.attitude_setpoint_publisher = self.node.create_publisher(
                VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        
        self.attitude_setpoint_publisher.publish(msg)
    
    def _publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish a vehicle command"""
        msg = VehicleCommand()
        msg.timestamp = int(time.time() * 1e6)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        self.vehicle_command_publisher.publish(msg)
        self.logger.info(f"Published vehicle command: {command}, params: {param1}, {param2}")
    
    def arm(self):
        """Arm the drone
        
        Returns:
            bool: Success flag
        """
        self.logger.info("Arming drone...")
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0  # 1 = arm
        )
        
        # Wait for arming
        timeout = time.time() + 3.0  # 3 second timeout
        while not self.armed and time.time() < timeout:
            time.sleep(0.1)
            
        if self.armed:
            self.logger.info("Drone armed successfully")
        else:
            self.logger.warning("Failed to arm drone")
            
        return self.armed
    
    def disarm(self):
        """Disarm the drone"""
        self.logger.info("Disarming drone...")
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0  # 0 = disarm
        )
    
    def enable_offboard_mode(self):
        """Enable offboard control mode
        
        Returns:
            bool: Success flag
        """
        self.logger.info("Enabling offboard control mode...")
        
        # First send setpoints at >2Hz for a while
        start_time = time.time()
        while time.time() - start_time < 1.0:
            self._publish_offboard_control_mode()
            self._publish_attitude_setpoint(0.0, 0.0, 0.0, 0.1)  # Minimal thrust to avoid drift
            time.sleep(0.05)  # 20Hz
        
        # Then request offboard mode
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,  # Mode
            6.0   # Custom mode - offboard
        )
        
        # Keep sending setpoints while waiting for mode switch
        timeout = time.time() + 5.0  # 5 second timeout
        while not self.offboard_mode and time.time() < timeout:
            self._publish_offboard_control_mode()
            self._publish_attitude_setpoint(0.0, 0.0, 0.0, 0.1)
            time.sleep(0.05)
        
        if self.offboard_mode:
            self.logger.info("Offboard mode enabled successfully")
        else:
            self.logger.warning("Failed to enable offboard mode")

        return self.offboard_mode
    
    def reset(self, seed=None, options=None):
        """Reset the environment for a new episode"""
        super().reset(seed=seed)
        
        # Make sure we're in offboard mode and armed
        if not self.offboard_mode:
            self.enable_offboard_mode()
        
        if not self.armed:
            self.arm()
        
        # Return current observation
        return self._get_obs(), {}
    
    def step(self, action):
        """Execute one step with direct roll, pitch, yaw rate, thrust control
        
        Args:
            action: array [roll, pitch, yaw_rate, thrust], values in [-1,1] range for roll, pitch, yaw_rate
                   and [0,1] range for thrust
        
        Returns:
            observation, reward, terminated, truncated, info
        """
        # Make sure we're still sending offboard control heartbeat at >2Hz
        current_time = time.time()
        if current_time - self.last_offboard_timestamp > 0.2:  # Send at least at 5Hz
            self._publish_offboard_control_mode()
        
        # Publish attitude setpoint
        roll = float(np.clip(action[0], -1.0, 1.0))
        pitch = float(np.clip(action[1], -1.0, 1.0))
        yaw_rate = float(np.clip(action[2], -1.0, 1.0))
        thrust = float(np.clip(action[3], 0.0, 1.0))
        
        self._publish_attitude_setpoint(roll, pitch, yaw_rate, thrust)
        
        # Add a small delay to allow for state update
        time.sleep(0.01)
        
        # Get observation
        observation = self._get_obs()
        
        # Simple check for terminated state - not armed or not in offboard mode
        terminated = not self.armed or not self.offboard_mode
        truncated = False
        
        # Simple reward - just staying alive
        reward = 0.0 if terminated else 0.1
        
        # Info dictionary
        info = {
            "armed": self.armed,
            "offboard_mode": self.offboard_mode
        }

        return observation, reward, terminated, truncated, info
    
    def _get_obs(self):
        """Get current observation"""
        return self.current_state.copy()
    
    def close(self):
        """Clean up resources"""
        self.logger.info("Closing environment")
        
        try:
            # Disarm if armed
            if self.armed:
                self.disarm()
            
            # Shutdown ROS node
            self.node.destroy_node()
            rclpy.shutdown()
            self.logger.info("ROS2 node shut down")
        except Exception as e:
            self.logger.error(f"Error during environment cleanup: {e}")