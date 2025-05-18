
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np
from drone_env import PX4DroneEnv
import gymnasium as gym
import cv2
import time



def main():
    env = PX4DroneEnv(headless=True)
    obs, _ = env.reset()

    observation_space = env.observation_space # Observation space: Box(0, 255, (240, 320, 3), uint8)
    action_space = env.action_space
    
    # Run for 10 seconds
    start_time = time.time()
    while time.time() - start_time < 10:
        # Sample a random action
        action = action_space.sample()
        
        # Take a step in the environment
        obs, reward, terminated, truncated, info = env.step(action)
        state = env.get_env_info()

        
        # Check if the episode is done
        if terminated or truncated:
            obs, _ = env.reset()
        
        # Convert the observation to a numpy array
        obs = np.array(obs)
        
        # Display the image stream using CV2
        #cv2.imshow("Drone Camera", obs)
        #cv2.waitKey(1)
        # Print image values
        print("Image shape:", obs.shape)
        print(obs)

    # Close the environment
    env.close()
    


if __name__ == "__main__":
    main()

