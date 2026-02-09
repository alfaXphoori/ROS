#!/usr/bin/env python3
"""Simple robot controller for learning."""

from controller import Robot

def main():
    # Initialize the robot
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    
    # Get motors
    left_motor = robot.getDevice('left_motor')
    right_motor = robot.getDevice('right_motor')
    
    # Set motors to velocity control mode
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    # Set initial velocities
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    print("Robot controller started!")
    print("Moving forward for 3 seconds...")
    
    # Main control loop
    time_counter = 0
    max_time = 3000  # 3 seconds in milliseconds
    
    while robot.step(timestep) != -1:
        if time_counter < max_time:
            # Move forward
            left_motor.setVelocity(2.0)
            right_motor.setVelocity(2.0)
        else:
            # Stop
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            print("Robot stopped.")
            break
        
        time_counter += timestep

if __name__ == '__main__':
    main()
