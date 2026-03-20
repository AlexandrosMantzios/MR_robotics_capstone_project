"""
27-05-2024
Capstone Project - Modern robotics
Next_State.py : Module to compute the next robot configuration on a given
set of velocities using Euler integration and odometry.

@ Alexandros Mantzios
"""

# Import necessary modules and classes
from Configurations import RobotConfiguration, RobotProperties
import numpy as np
import csv

# Define constants
speed_max = 5  # Maximum speed for wheels


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
        return angle_new
    except Exception as e:
        print(f"Error in euler_step function: {e}")
        return None


def odometry(chassis_conf, Dt_wheel_angles, F):
    """
    Odometry function to calculate chassis displacement from wheel angle changes.
    Inputs:
    - chassis_conf : Chassis configuration (phi, x, y)
    - Dt_wheel_angles : Change in wheel angles (4 elements)
    - F : Pre-computed 3x4 mobile base kinematics matrix (pseudoinverse of H0)
    Outputs:
    - D_conf : Chassis displacement in space frame [dphi, dx, dy]
    """
    try:
        # Ensure Dt_wheel_angles has 4 elements
        if Dt_wheel_angles.shape[0] != 4:
            raise ValueError(f"Expected Dt_wheel_angles to have 4 elements, got {Dt_wheel_angles.shape[0]}")

        Vb = np.dot(F, Dt_wheel_angles)  # Calculate body velocity [wz, vx, vy]
        w_bz = Vb[0]  # Extract rotational velocity
        v_bx = Vb[1]  # Extract x-axis velocity
        v_by = Vb[2]  # Extract y-axis velocity
        phi = chassis_conf[0]  # Extract the orientation angle (phi)

        # Calculate change in configuration based on velocities
        # Use exact integration (arc) when rotating, straight line when not
        if abs(w_bz) < 1e-12:
            D_conf_b = np.array([0, v_bx, v_by])
        else:
            D_conf_b = np.array([w_bz,
                                 (v_bx * np.sin(w_bz) + v_by * (np.cos(w_bz) - 1)) / w_bz,
                                 (v_by * np.sin(w_bz) + v_bx * (1 - np.cos(w_bz))) / w_bz])

        # Rotation matrix to transform body displacement to space frame
        H = np.array([[1, 0, 0],
                       [0, np.cos(phi), -np.sin(phi)],
                       [0, np.sin(phi), np.cos(phi)]])
        D_conf = np.dot(H, D_conf_b)  # Transform to space frame
        return D_conf
    except Exception as e:
        print(f"Error in odometry function: {e}")
        return None


def NextState(initial_config, velocities, joint_speed_limit, Dt, F,
              chassis_speed_max=speed_max):
    """
    Next State function to calculate next configuration.
    Inputs:
    - initial_config : Initial configuration (instance of RobotConfiguration)
    - velocities : Velocities [joint1..5, wheel1..4] (9 elements)
    - joint_speed_limit : Joint speed limit as defined in Parameters
    - Dt : Time step
    - F : Pre-computed 3x4 mobile base kinematics matrix
    - chassis_speed_max : Chassis max speed (default: speed_max)
    Outputs:
    - new_config : Next configuration (RobotConfiguration, 13 elements)
    """
    try:
        joint_speed_max = joint_speed_limit
        chassis_conf = initial_config.get_chassi_configuration()  # Get initial chassis [phi, x, y]
        joint_angles = initial_config.get_joint_angles()  # Get initial joint angles [j1..j5]
        wheel_angles = initial_config.get_wheel_angles()  # Get initial wheel angles [w1..w4]

        # Split velocities into joint and wheel components
        joint_velocities = velocities[:5]
        wheel_velocities = velocities[5:]

        # Calculate change in wheel angles for odometry
        Dt_wheel_angles = Dt * wheel_velocities
        # Update wheel angles via Euler step
        wheel_angles = euler_step(wheel_angles, wheel_velocities, chassis_speed_max, Dt)
        if wheel_angles is None:
            raise ValueError("Error in euler_step for wheel angles")

        # Update chassis configuration via odometry
        odom_result = odometry(chassis_conf, Dt_wheel_angles, F)
        if odom_result is None:
            raise ValueError("Error in odometry function")
        chassis_conf = chassis_conf + odom_result

        # Update joint angles via Euler step
        joint_angles = euler_step(joint_angles, joint_velocities, joint_speed_max, Dt)
        if joint_angles is None:
            raise ValueError("Error in euler_step for joint angles")

        # Concatenate new configuration: [chassis, joints, wheels]
        new_config_values = np.concatenate([chassis_conf, joint_angles, wheel_angles])
        new_config = RobotConfiguration(new_config_values)
        return new_config
    except Exception as e:
        print(f"Error in NextState function: {e}")
        return None


def generate_row(config, gripper_state):
    """
    Function to add the configuration data in a row + gripper state
    for CoppeliaSim animation CSV output.
    Inputs:
    - config : Current configuration (RobotConfiguration)
    - gripper_state : State of the gripper (0=open, 1=closed)
    Outputs:
    - row : 14-element array [config + gripper]
    """
    try:
        row = np.concatenate([config.get_configuration_as_array_(), [gripper_state]])
        return row
    except Exception as e:
        print(f"Error in generate_row function: {e}")
        return None


def write_to_csv(row_list, fn):
    """
    Function to write the configuration data to a CSV file.
    Inputs:
    - row_list : List of rows to be written
    - fn : Filename for the CSV file
    """
    try:
        with open(fn, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=',')
            for row in row_list:
                writer.writerow(row)
    except Exception as e:
        print(f"Error in write_to_csv function: {e}")
