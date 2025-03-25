#!/usr/bin/env python3
"""
Test script for simplified PX4DroneEnv gymnasium environment
Tests basic drone control capabilities including offboard control, arming, and basic flight
"""
import time
import argparse
import numpy as np
from drone_env import PX4DroneEnv
import signal
import sys

# Global variable for environment instance to handle graceful shutdown
env = None

def signal_handler(sig, frame):
    """Handle CTRL+C to gracefully close the environment"""
    print("\nReceived interrupt, closing environment...")
    if env is not None:
        env.close()
    sys.exit(0)

def test_basic_flight():
    """Test offboard control, arming, and basic flight maneuvers"""
    global env
    env = PX4DroneEnv(debug=True)
    
    print("\n=== Starting test: Offboard Mode, Arming and Flight Control ===")
    
    # Enable offboard control first - this is required before arming for offboard flight
    print("Enabling offboard control mode...")
    success = env.enable_offboard_mode()
    if not success:
        print("Failed to enter offboard control mode. Exiting test.")
        env.close()
        return
    print("Successfully entered offboard control mode!")
    
    # Arm the drone
    print("Arming drone...")
    success = env.arm()
    if not success:
        print("Failed to arm the drone. Exiting test.")
        env.close()
        return
    print("Successfully armed the drone!")
    
    # Reset environment to get initial observation
    print("Initializing environment...")
    obs, _ = env.reset()
    initial_position = obs[0:3].copy()
    initial_orientation = obs[6:9].copy()
    print(f"Initial position: x={initial_position[0]:.2f}, y={initial_position[1]:.2f}, z={initial_position[2]:.2f}")
    print(f"Initial orientation: roll={initial_orientation[0]:.2f}, pitch={initial_orientation[1]:.2f}, yaw={initial_orientation[2]:.2f}")
    
    # Hold position with slight thrust to stabilize
    print("\nPhase 1: Stabilizing with moderate thrust...")
    for i in range(30):  # 3 seconds at 0.1s per step
        action = np.array([0.0, 0.0, 0.0, 0.4])  # Neutral controls with moderate thrust
        obs, reward, terminated, truncated, info = env.step(action)
        if i % 10 == 0:  # Print position every second
            print(f"Position: x={obs[0]:.2f}, y={obs[1]:.2f}, z={obs[2]:.2f}")
        if terminated or truncated:
            print("Flight ended early")
            break
        time.sleep(0.1)
    
    # Test increasing thrust
    print("\nPhase 2: Testing thrust increase...")
    for i in range(30):  # 3 seconds
        # Gradually increase thrust
        thrust = 0.4 + (i / 20) * 0.3  # Increase from 0.4 to 0.7
        action = np.array([0.0, 0.0, 0.0, thrust])
        obs, reward, terminated, truncated, info = env.step(action)
        if i % 5 == 0:
            print(f"Thrust: {thrust:.2f}, Position: x={obs[0]:.2f}, y={obs[1]:.2f}, z={obs[2]:.2f}")
        if terminated or truncated:
            print("Flight ended early")
            break
        time.sleep(0.1)
    
    # Test pitch (forward/backward movement)
    print("\nPhase 3: Testing pitch control (forward then backward)...")
    # Forward movement
    for i in range(30):  # 3 seconds
        action = np.array([0.0, -0.6, 0.0, 0.7])  # Pitch forward with moderate thrust
        obs, reward, terminated, truncated, info = env.step(action)
        if i % 5 == 0:
            print(f"Forward pitch: -0.3, Position: x={obs[0]:.2f}, y={obs[1]:.2f}, z={obs[2]:.2f}")
        if terminated or truncated:
            print("Flight ended early")
            break
        time.sleep(0.1)
    
    # Backward movement
    for i in range(30):  # 3 seconds
        action = np.array([0.0, 0.6, 0.0, 0.7])  # Pitch backward with moderate thrust
        obs, reward, terminated, truncated, info = env.step(action)
        if i % 5 == 0:
            print(f"Backward pitch: 0.3, Position: x={obs[0]:.2f}, y={obs[1]:.2f}, z={obs[2]:.2f}")
        if terminated or truncated:
            print("Flight ended early")
            break
        time.sleep(0.1)
    
    # Stabilize briefly
    print("\nStabilizing...")
    for i in range(10):
        env.step(np.array([0.0, 0.0, 0.0, 0.4]))
        time.sleep(0.1)
    
    # Test roll (left/right movement)
    print("\nPhase 4: Testing roll control (right then left)...")
    # Right movement
    for i in range(30):  # 3 seconds
        action = np.array([0.6, 0.0, 0.0, 0.7])  # Roll right with moderate thrust
        obs, reward, terminated, truncated, info = env.step(action)
        if i % 5 == 0:
            print(f"Right roll: 0.3, Position: x={obs[0]:.2f}, y={obs[1]:.2f}, z={obs[2]:.2f}")
        if terminated or truncated:
            print("Flight ended early")
            break
        time.sleep(0.1)
    
    # Left movement
    for i in range(30):  # 3 seconds
        action = np.array([-0.6, 0.0, 0.0, 0.7])  # Roll left with moderate thrust
        obs, reward, terminated, truncated, info = env.step(action)
        if i % 5 == 0:
            print(f"Left roll: -0.3, Position: x={obs[0]:.2f}, y={obs[1]:.2f}, z={obs[2]:.2f}")
        if terminated or truncated:
            print("Flight ended early")
            break
        time.sleep(0.1)
    
    # Stabilize briefly
    print("\nStabilizing...")
    for i in range(10):
        env.step(np.array([0.0, 0.0, 0.0, 0.4]))
        time.sleep(0.1)
    
    # Test yaw rate (rotation)
    print("\nPhase 5: Testing yaw rate control (clockwise then counter-clockwise)...")
    # Clockwise rotation
    for i in range(30):  # 3 seconds
        action = np.array([0.0, 0.0, 0.6, 0.7])  # Yaw clockwise with moderate thrust
        obs, reward, terminated, truncated, info = env.step(action)
        if i % 5 == 0:
            print(f"Clockwise yaw rate: 0.4, Yaw: {obs[8]:.2f} radians")
        if terminated or truncated:
            print("Flight ended early")
            break
        time.sleep(0.1)
    
    # Counter-clockwise rotation
    for i in range(30):  # 3 seconds
        action = np.array([0.0, 0.0, -0.6, 0.7])  # Yaw counter-clockwise with moderate thrust
        obs, reward, terminated, truncated, info = env.step(action)
        if i % 5 == 0:
            print(f"Counter-clockwise yaw rate: -0.4, Yaw: {obs[8]:.2f} radians")
        if terminated or truncated:
            print("Flight ended early")
            break
        time.sleep(0.1)
    
    # Test combined controls (complex maneuver)
    print("\nPhase 6: Testing combined control inputs...")
    for i in range(40):  # 4 seconds
        # Create different combinations of controls
        if i < 5:
            action = np.array([0.6, -0.6, 0.3, 0.7])  # Roll right + pitch forward + slight yaw
        elif i < 10:
            action = np.array([-0.6, -0.6, -0.3, 0.8])  # Roll left + pitch forward + opposite yaw + more thrust
        elif i < 15:
            action = np.array([0.6, 0.6, 0.6, 0.8])  # Roll right + pitch backward + yaw clockwise
        else:
            action = np.array([-0.6, 0.6, -0.6, 0.7])  # Roll left + pitch backward + yaw counter-clockwise
            
        obs, reward, terminated, truncated, info = env.step(action)
        if i % 5 == 0:
            print(f"Combined action: {action}, Position: x={obs[0]:.2f}, y={obs[1]:.2f}, z={obs[2]:.2f}, Yaw: {obs[8]:.2f}")
        if terminated or truncated:
            print("Flight ended early")
            break
        time.sleep(0.1)
    
    # Hold position before ending
    print("\nPhase 7: Stabilizing before landing...")
    for i in range(40):  # 4 seconds at 0.1s per step
        action = np.array([0.0, 0.0, 0.0, 0.4])  # Neutral controls with moderate thrust
        obs, reward, terminated, truncated, info = env.step(action)
        if i % 10 == 0:
            print(f"Position: x={obs[0]:.2f}, y={obs[1]:.2f}, z={obs[2]:.2f}")
        if terminated or truncated:
            print("Flight ended early")
            break
        time.sleep(0.1)
    
    # Report final position and distance traveled
    final_position = obs[0:3].copy()
    distance = np.linalg.norm(final_position - initial_position)
    print(f"\nFlight complete. Total distance from start: {distance:.2f}m")
    print(f"Final position: x={obs[0]:.2f}, y={obs[1]:.2f}, z={obs[2]:.2f}")
    
    # Disarm and close environment
    print("Disarming drone...")
    env.disarm()
    time.sleep(1.0)
    
    env.close()
    print("=== Test completed: Offboard Mode, Arming and Flight Control ===\n")

if __name__ == "__main__":
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    parser = argparse.ArgumentParser(description="Test the simplified PX4 drone environment")
    parser.add_argument('--test', type=str, default='basic', 
                        choices=['all', 'basic', 'limits'],
                        help='Which test to run')
    
    args = parser.parse_args()
    
    try:
        if args.test == 'all' or args.test == 'basic':
            test_basic_flight()
            time.sleep(2)
            
        if args.test == 'all' or args.test == 'limits':
            # Placeholder for future limit tests
            print("Limit tests not implemented yet.")
            time.sleep(2)
            
    except KeyboardInterrupt:
        print("\nTests interrupted by user")
        if env is not None:
            print("Closing environment...")
            env.close()
    except Exception as e:
        print(f"Error during test: {e}")
        if env is not None:
            env.close()