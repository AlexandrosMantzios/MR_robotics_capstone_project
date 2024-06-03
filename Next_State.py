"""
27-05-2024
Capstone Project - Modern robotics
Next_State.py : Module to compute the next robot configuration on a given 

@ Alexandros Mantzios

"""

# Import necessary modules and classes
from Configurations import RobotConfiguration, RobotProperties
import os
import numpy as np
import math as m
import csv
from numpy.linalg import inv

# Re-import RobotConfiguration and RobotProperties to ensure availability
from Configurations import RobotConfiguration, RobotProperties
import numpy as np

# Define constants
Dt = 0.01  # Time step for integration
speed_max = 5  # Maximum speed

def euler_step(angle_current, speed_new, speed_max, Dt):
    """
    Function to perform euler_step calculation
    Inputs : 
    - angle_current : Current angle values
    - speed_new : New velocity
    - speed_max : Max velocity
    - Dt : Time interval
    Outputs: 
    - New angle value
    """
    try:
        angle_current = np.array(angle_current)  # Convert current angle to numpy array
        speed_new = np.array(speed_new)  # Convert new speed to numpy array
        speed_clipped = np.clip(speed_new, -speed_max, speed_max)  # Clip speeds to max speed limits
        min_size = min(len(angle_current), len(speed_clipped))  # Get the minimum size between angles and speeds
        angle_current = angle_current[:min_size]  # Slice the angle array to the minimum size
        speed_clipped = speed_clipped[:min_size]  # Slice the speed array to the minimum size
        angle_new = angle_current + speed_clipped * Dt  # Calculate new angle using Euler method
        return angle_new  # Return the new angle
    except Exception as e:
        print(f"Error in euler_step function: {e}")
        return None

def odometry(chassis_conf, Dt_wheel_angles):
    """
    Odometry function to calculate odometry for mobile robot 
    Inputs:
    - chassis_conf : Chassis configuration (phi, x, y)
    - Dt_wheel_angles : Change in wheel angles
    Outputs: 
    - Updated chassis configuration
    """
    try:
        chassi_properties = RobotProperties()  # Instantiate RobotProperties
        r = chassi_properties.r  # Get wheel radius
        l = chassi_properties.l  # Get distance to wheel axis
        ω = chassi_properties.ω  # Get half wheel axis
        # Define the H0 matrix for the mobile robot
        H0 = (1/r) * np.array([[-l-ω, 1, -1],
                               [l+ω, 1, 1],
                               [l+ω, 1, -1],
                               [-l-ω, 1, 1]])
        F = np.linalg.pinv(H0)  # Calculate pseudoinverse of H0
        # Ensure Dt_wheel_angles has 4 elements
        if Dt_wheel_angles.shape[0] != 4:
            raise ValueError(f"Expected Dt_wheel_angles to have 4 elements, got {Dt_wheel_angles.shape[0]}")
        Vb = np.dot(F, Dt_wheel_angles)  # Calculate body velocity
        w_bz = Vb[0]  # Extract rotational velocity
        v_bx = Vb[1]  # Extract x-axis velocity
        v_by = Vb[2]  # Extract y-axis velocity
        φ = chassis_conf[0]  # Extract the orientation angle (phi)
        # Calculate change in configuration based on velocities
        if abs(w_bz) < 1e-12:
            D_conf_b = np.array([0, v_bx, v_by])
        else:
            D_conf_b = np.array([w_bz, (v_bx*np.sin(w_bz)+v_by*(np.cos(w_bz)-1))/w_bz, 
                                (v_by*np.sin(w_bz)+v_bx*(1-np.cos(w_bz)))/w_bz])
        # Define rotation matrix for the chassis
        H = np.array([[1, 0, 0],
                      [0, np.cos(φ), -np.sin(φ)],
                      [0, np.sin(φ), np.cos(φ)]])
        D_conf = np.dot(H, D_conf_b)  # Calculate new chassis configuration
        return D_conf  # Return the updated chassis configuration
    except Exception as e:
        print(f"Error in odometry function: {e}")
        return None

def NextState(initial_config, velocities, joint_speed_limit, chassis_speed_max=speed_max):
    """
    Next State function to calculate next state - next configuration
    Inputs: 
    - initial_config : Initial configuration (instance of RobotConfiguration)
    - velocities : Velocities (4 wheel and 5 joint angles)
    - joint_speed_limit : Joint speed limit as defined in Parameters (degrees/second)
    - chassis_speed_max : Chassis max speed as defined in Parameters (meters/second)
    Outputs:
    - new_config : Next configuration (13 elements)
    """
    try:
        joint_speed_max = joint_speed_limit  # Set maximum joint speed
        chassis_conf = initial_config.get_chassi_configuration()  # Get initial chassis configuration
        joint_angles = initial_config.get_joint_angles()  # Get initial joint angles
        wheel_angles = initial_config.get_wheel_angles()  # Get initial wheel angles

        # Ensure the correct slice for joint and wheel angles
        joint_velocities = velocities[:5]  # Extract joint velocities
        wheel_velocities = velocities[5:]  # Extract wheel velocities

        Dt_wheel_angles = Dt * wheel_velocities  # Calculate change in wheel angles
        wheel_angles = euler_step(wheel_angles, wheel_velocities, chassis_speed_max, Dt)  # Update wheel angles
        if wheel_angles is None:
            raise ValueError("Error in euler_step for wheel angles")
        
        chassis_conf = chassis_conf + odometry(chassis_conf, Dt_wheel_angles)  # Update chassis configuration
        if chassis_conf is None:
            raise ValueError("Error in odometry function")

        joint_angles = euler_step(joint_angles, joint_velocities, joint_speed_max, Dt)  # Update joint angles
        if joint_angles is None:
            raise ValueError("Error in euler_step for joint angles")

        new_config_values = np.concatenate([chassis_conf, joint_angles, wheel_angles])  # Concatenate new configuration values
        new_config = RobotConfiguration(new_config_values)  # Create new RobotConfiguration instance
        return new_config  # Return the new configuration
    except Exception as e:
        print(f"Error in NextState function: {e}")
        return None

def generate_row(config, gripper_state):
    """
    Function to add the configuration data in a row + gripper state
    Inputs:
    - config : Current configuration
    - gripper_state : State of the gripper (open/closed)
    Outputs:
    - row : Combined configuration data and gripper state
    """
    try:
        row = np.concatenate([config.get_configuration_as_array_(), [gripper_state]])  # Combine configuration and gripper state
        return row  # Return the combined row
    except Exception as e:
        print(f"Error in generate_row function: {e}")
        return None

def write_to_csv(row_list, fn):
    """
    Function to write the data in csv file
    Inputs:
    - row_list : List of rows to be written
    - fn : Filename for the CSV file
    """
    try:
        with open(fn, 'w', newline='') as csvfile:  # Open the CSV file for writing
            writer = csv.writer(csvfile, delimiter=',')  # Create a CSV writer object
            for row in row_list:  # Iterate through each row in the list
                writer.writerow(row)  # Write the row to the CSV file
    except Exception as e:
        print(f"Error in write_to_csv function: {e}")



