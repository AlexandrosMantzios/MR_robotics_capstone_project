"""
01-06-2024
Capstone Project - Modern robotics
main.py : Combines Next_State.py, FeedbackController.py and Trajectories.py
and outputs a result csv file for the CoppeliaSim animation.

Basic functionality
- Calculates the control law using FeedbackControl and generates control velocities
- Sends controls, configuration and timestep to NextState and calculates new configuration.
- Stores configurations for later animations
- Shows error over time

@ Alexandros Mantzios
v4.0
"""

# Import necessary modules
import sys
import logging
import matplotlib.pyplot as plt
import numpy as np
import os
from Next_State import NextState, generate_row, write_to_csv
from FeedbackController import Controller
from IPython import get_ipython
from Configurations import RobotConfiguration, RobotProperties, Cube_Task, Parameters
from Trajectories import TrajectoryPlanner
from tqdm import tqdm
import datetime

# Setup logging to file (console output remains available for tqdm and errors)
logging.basicConfig(filename="logfile.txt", level=logging.DEBUG,
                    format='%(asctime)s %(message)s', filemode='w')
logger = logging.getLogger(__name__)

# Prompt the user to input the task to run
while True:
    task = input("Please enter the task to run (best, overshoot, newtask): ").strip().lower()
    if task in ["best", "overshoot", "newtask"]:
        break
    else:
        print("Invalid input. Please enter one of the following: best, overshoot, newtask.")

# Initialize robot properties (single instance, shared with Controller and NextState)
robot_props = RobotProperties()
cube_task = Cube_Task()

# Load task geometry based on selected task
if task == "newtask":
    Tsc_ini = cube_task.Tsc_init_new()   # Initial cube configuration - new task
    Tsc_goal = cube_task.Tsc_goal_new()  # Goal cube configuration - new task
else:
    Tsc_ini = cube_task.Tsc_init()       # Initial cube configuration - best/overshoot
    Tsc_goal = cube_task.Tsc_goal()      # Goal cube configuration - best/overshoot

Tce_grip = cube_task.Tce_grip()          # Gripper configuration (EE at cube)
Tce_standoff = cube_task.Tce_standoff()  # Pre-grip configuration (EE above cube)
Tse_init = robot_props.Tse_init          # Initial end-effector configuration


def store_xerr(Xerr_values, task):
    """
    Stores X_err values to CSV file for later analysis.
    """
    try:
        filepath = os.path.dirname(os.path.abspath(__file__))
        filename = os.path.join(filepath, f"Xerr_{task}.csv")
        np.savetxt(filename, Xerr_values, delimiter=",")
    except Exception as e:
        print(f"Error storing X_err values: {e}")


def Xerr_plot(Xerr_values, Kp, Ki):
    """
    Plotter for Xerr with the use of IPython.
    Shows 6-DOF error over time.
    """
    try:
        ipython = get_ipython()  # Get IPython instance
        if ipython:
            ipython.run_line_magic('matplotlib', 'qt')  # Set IPython to use Qt backend for matplotlib
    except NameError:
        pass

    try:
        # Plot each error component with meaningful labels
        labels = ["wx", "wy", "wz", "vx", "vy", "vz"]
        for idx, label in enumerate(labels):
            plt.plot(Xerr_values[:, idx], label=label)

        # Set legend and title
        plt.legend()
        plt.title(f"P: {Kp[0, 0]} I: {Ki}")
        plt.xlabel("Timestep")
        plt.ylabel("Error")

        # Display the plot
        plt.show()
    except Exception as e:
        print(f"Error plotting X_err values: {e}")


if __name__ == "__main__":
    """
    Main for running the script with the robot task
    """
    try:
        np.set_printoptions(suppress=True, precision=3)  # Suppress scientific notation
        logger.info(datetime.datetime.now())
        logger.info("Starting the main script...")

        # Define initial robot configuration (13 elements)
        # [phi, x, y, j1, j2, j3, j4, j5, w1, w2, w3, w4, gripper]
        initial_config_values = np.array([np.pi / 4, -0.5, 0.5, 1, -0.2, 0.2, -1.6, 0, 0, 0, 0, 0, 0])
        initial_config = RobotConfiguration(initial_config_values)

        # Create controller instance (caches robot geometry for all subsequent calls)
        controller = Controller(robot_props)

        # Set control gains based on task
        Xerr_list = []  # Collect errors as list (converted to array at end, avoids O(N^2) concatenation)
        if task == "best":
            Kp = np.eye(6) * 0.75  # Set proportional gain
            Ki = 0.0               # Set integral gain
        elif task == "overshoot":
            Kp = np.eye(6) * 2.0   # Set proportional gain (higher causes overshoot)
            Ki = 2.0               # Set integral gain (non-zero causes overshoot)
        elif task == "newtask":
            Kp = np.eye(6) * 1.0   # Set proportional gain
            Ki = 0.0               # Set integral gain

        Ki_err = 0.0  # Initialize integral error accumulator
        k = 1         # Set scaling factor
        Dt = 0.01 / k  # Set time step
        velocity_max = Parameters.max_linear_velocity       # Get maximum linear velocity
        joint_speed_limit = Parameters.max_angular_velocity  # Get maximum angular velocity
        angular_velocity_max = Parameters.max_angular_velocity  # For trajectory generation

        # Generate full 8-segment pick-and-place trajectory
        Full_trajectory_list, Full_gripper_list = TrajectoryPlanner.Full_trajectory(
            Tsc_ini, Tsc_goal, Tse_init, Tce_standoff, Tce_grip,
            k, velocity_max, angular_velocity_max)

        all_configuration_data_rows = []  # List to store all data rows for CSV

        # Outer loop: iterate over each trajectory segment
        for i, trajectory_segment in tqdm(enumerate(Full_trajectory_list), position=0):
            reference_trajectory = Full_trajectory_list[i]  # Get the current trajectory segment
            gripper_state = Full_gripper_list[i]            # Get the current gripper state
            configuration_data_rows = []

            # Inner loop: iterate over each step in the trajectory segment
            for j in tqdm(range(len(reference_trajectory) - 1), position=0):
                # Generate a data row for the current configuration and gripper state
                configuration_data_row = generate_row(initial_config, gripper_state)
                configuration_data_rows.append(configuration_data_row)

                # Perform joint limit test
                Joint_limits_test = controller.Joint_Limit_test(initial_config, ignore=False)

                # Get current and next desired end-effector configurations
                Tse_d = reference_trajectory[j]
                Tse_d_next = reference_trajectory[j + 1]

                # Perform feedback control - compute velocities
                controls, Xerr, Ki_err, J_robot, Twist, Vd = controller.FeedbackControl(
                    initial_config, Tse_d, Tse_d_next, Kp, Ki, Ki_err, Dt, Joint_limits_test)

                # Store error for plotting
                Xerr_list.append(Xerr)

                # Extract and reorder velocities:
                # FeedbackControl returns [wheel1..4, joint1..5]
                # NextState expects [joint1..5, wheel1..4]
                wheel_velocities = np.array(controls[0:4])
                joint_velocities = np.array(controls[4:])
                combined_velocities = np.concatenate((joint_velocities, wheel_velocities))

                # Calculate the next state using Euler integration and odometry
                final_configuration = NextState(
                    initial_config, combined_velocities, joint_speed_limit, Dt, robot_props.F)
                initial_config = final_configuration  # Update for next iteration

            # Collect data rows for all segments
            all_configuration_data_rows.extend(configuration_data_rows)

        # Write all configuration data to CSV for CoppeliaSim animation
        write_to_csv(all_configuration_data_rows, fn=f'result_{task}.csv')

        # Convert error list to array and output results
        Xerr_array = np.array(Xerr_list)
        logger.info("Producing error Plot")
        logger.info(f"P: {Kp[0, 0]} I: {Ki}")
        store_xerr(Xerr_array, task)
        Xerr_plot(Xerr_array, Kp, Ki)

        logger.info("Script Completed")
    except Exception as e:
        print(f"Error in main script: {e}")
        sys.exit(1)
