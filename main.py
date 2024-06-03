"""
01-06-2024
Capstone Project - Modern robotics
main.py : Combines  Next_State.py, FeedbackController.py and Trajectories.py
and outputs a result_AM csv file for the coppelia animation. 

Basic functionality 
- Calculates the control law using FeedbackControl and generates control velocities
- Sends controls, configuration and timestep to NextState and calculates new configuration.
- Stores configurations for later animations 
- Shows error over time
   

@ Alexandros Mantzios

"""
             
# Import necessary modules
import sys  # System-specific parameters and functions
import matplotlib.pyplot as plt  # Plotting library
import numpy as np  # Numerical operations
import os  # Operating system interface
import math  # Mathematical functions
from numpy.linalg import inv  # Linear algebra operations
from scipy.spatial.transform import Rotation  # Spatial transformations
from Next_State import NextState, generate_row, write_to_csv  # Custom module for next state calculations and file operations
from FeedbackController import Controller  # Custom module for feedback control
from IPython import get_ipython  # IPython-specific operations
import Configurations  # Custom configurations module
from Configurations import RobotConfiguration, RobotProperties, Cube_Task, Parameters  # Import specific classes from Configurations
import csv  # CSV file operations
from Trajectories import TrajectoryPlanner  # Custom module for trajectory planning
import modern_robotics as mr  # Modern Robotics library
from tqdm import tqdm  # Progress bar for loops
import datetime  # Date and time functions

# Ensure output is flushed immediately
sys.stdout.flush()

# Prompt the user to input the task to run
while True:
    task = input("Please enter the task to run (best, overshoot, newtask): ").strip().lower()
    if task in ["best", "overshoot", "newtask"]:
        break
    else:
        print("Invalid input. Please enter one of the following: best, overshoot, newtask.")

try:
    # Initialization
    cube_task = Cube_Task()  # Initialize Cube_Task
    robot_parameters = Parameters()  # Initialize Parameters
    robot_properties = RobotProperties()  # Initialize RobotProperties
    initial_config = RobotConfiguration(np.array([np.pi / 4, -0.5, 0.5, 1, -0.2, 0.2, -1.6, 0, 0, 0, 0, 0, 0]))  # Set initial robot configuration
    Tsc_ini = cube_task.Tsc_init()  # Initial cube configuration

    if task == "best":
        Tsc_goal = cube_task.Tsc_goal()  # Goal cube configuration - best/overshoot
    elif task == "newtask":
        Tsc_goal = cube_task.Tsc_goal_new()  # Goal cube configuration - newtask
    elif task == "overshoot":
        Tsc_goal = cube_task.Tsc_goal()
    
    # # Debugging step
    # print(Tsc_goal)

    Tce_grip = cube_task.Tce_grip()  # Gripper configuration
    Tce_pre_grip = cube_task.Tce_standoff()  # Pre-grip configuration
    Tse_init = RobotProperties.Tse_init()  # Initial end-effector configuration
    Blist = RobotProperties.Blist()  # List of screw axes
    M_home = RobotProperties.M_Home()  # Home configuration
    Tbo = RobotProperties.Tb0()  # Base frame
    solver_time_scale = robot_parameters.time_scale  # Time scale for the solver
    Trajectory_type = robot_parameters.trajectory_type  # Type of trajectory
    gripper_open = 0  # Gripper open state
    gripper_closed = 1  # Gripper closed state
except AttributeError as e:
    print(f"Error during initialization: {e}")
    sys.exit(1)

try:
    # Write log file 
    f = open("logfile.txt", 'w')  # Open a log file for writing
    sys.stdout = f  # Redirect stdout to the log file
except IOError as e:
    print(f"Error opening log file: {e}")
    sys.exit(1)

def store_xerr(Xerr_values):
    """
    Stores X_err values 
    """
    try:
        filepath = os.path.dirname(os.path.abspath(__file__))  # Get current file directory
        if task == "best":
            filename = os.path.join(filepath, "Xerr_best.csv")  # Create the full path for the CSV file
            np.savetxt(filename, Xerr_values, delimiter=",")  # Save the X_err values to a CSV file
        elif task == "overshoot":
            filename = os.path.join(filepath, "Xerr_overshoot.csv")  # Create the full path for the CSV file
            np.savetxt(filename, Xerr_values, delimiter=",")
        elif task == "newtask":
            filename = os.path.join(filepath, "Xerr_newtask.csv")  # Create the full path for the CSV file
            np.savetxt(filename, Xerr_values, delimiter=",")
    except Exception as e:
        print(f"Error storing X_err values: {e}")

def Xerr_plot(Xerr_values, Kp, Ki):
    """
    Plotter for Xerr with the use of IPython
    """
    try:
        ipython = get_ipython()  # Get IPython instance
        if ipython:
            ipython.run_line_magic('matplotlib', 'qt')  # Set IPython to use Qt backend for matplotlib
    except NameError:
        pass
    
    try:
        # Plot each column of Xerr_values and collect labels
        labels = list(range(Xerr_values.shape[1]))  # Create labels for each X_err column
        for idx, label in enumerate(labels):
            plt.plot(Xerr_values[:, idx], label=str(label))  # Plot each X_err column
        
        # Set legend and title
        plt.legend()
        plt.title(f"P: {Kp[0, 0]} I: {Ki}")
        
        # Display the plot
        plt.show()
    except Exception as e:
        print(f"Error plotting X_err values: {e}")

if __name__ == "__main__":
    """
    Main for running the script with the robot task
    """
    try:
        file_dir = os.path.dirname(__file__)  # Get the directory of the current file
        sys.path.append(file_dir)  # Add the directory to the system path
        pi = math.pi  # Define pi
        np.set_printoptions(suppress=True)  # Suppress scientific notation in numpy output
        np.set_printoptions(precision=3)  # Set precision for numpy output
        print(datetime.datetime.now())  # Print the current date and time
        sys.stdout.flush()  # Flush stdout

        print("Starting the main script...")  # Print start message
        sys.stdout.flush()  # Flush stdout

        initial_config_values = np.array([np.pi / 4, -0.5, 0.5, 1, -0.2, 0.2, -1.6, 0, 0, 0, 0, 0, 0])  # Define initial configuration values
        initial_config = RobotConfiguration(initial_config_values)  # Initialize RobotConfiguration
        sys.stdout.flush()  # Flush stdout
        
        Tse_d = Controller.Tse_calc(initial_config.get_chassi_configuration(), initial_config.get_joint_angles())  # Calculate initial Tse_d
        sys.stdout.flush()  # Flush stdout

        Xerr_list = np.zeros((1, 6))  # Initialize Xerr_list with the correct dimensions
        if task == "best":
            Kp = np.eye(6) * 0.75  # Set proportional gain
            Ki = 0.0  # Set integral gain
        elif task == "overshoot":
            Kp = np.eye(6) * 2.0  # Set proportional gain
            Ki = 2.0  # Set integral gain
        elif task == "newtask":
            Kp = np.eye(6)*1.0 # 
            Ki = 0.0 # Set integral gain 

        # print (Kp,Ki)

        Ki_err = 0.0  # Initialize integral error
        k = 1  # Set scaling factor
        Dt = 0.01 / k  # Set time step
        velocity_max = Parameters.max_linear_velocity  # Get maximum linear velocity
        joint_speed_limit = Parameters.max_angular_velocity  # Get maximum angular velocity

        Full_trajectory_list, Full_gripper_list = TrajectoryPlanner.Full_trajectory(Tsc_ini, Tsc_goal, Tse_init, k, velocity_max)  # Generate full trajectory and gripper lists
        sys.stdout.flush()  # Flush stdout

        all_configuration_data_rows = []  # List to store all data rows

        for i, trajectory_segment in tqdm(enumerate(Full_trajectory_list), position=0):  # Iterate over each trajectory segment
            sys.stdout.flush()  # Flush stdout
            reference_trajectory = Full_trajectory_list[i]  # Get the current trajectory segment
            gripper_state = Full_gripper_list[i]  # Get the current gripper state
            configuration_data_rows = []  # List to store configuration data rows for the current segment

            for j in tqdm(range(len(reference_trajectory) - 2), position=0):  # Iterate over each step in the trajectory segment
                sys.stdout.flush()  # Flush stdout
                configuration_data_row = generate_row(initial_config, gripper_state)  # Generate a data row for the current configuration and gripper state
                configuration_data_rows.append(configuration_data_row)  # Add the data row to the list
                Joint_limits_test = Controller.Joint_Limit_test(initial_config, ignore=False)  # Perform joint limit test
                Tse_d = reference_trajectory[j]  # Get the current desired end-effector configuration
                Tse_d_next = reference_trajectory[j + 1]  # Get the next desired end-effector configuration

                controls, Xerr, Ki_err, J_robot, Twist, Vd = Controller.FeedbackControl(
                    initial_config, Tse_d, Tse_d_next, Kp, Ki, Ki_err, Dt, Joint_limits_test)  # Perform feedback control

                Xerr_list = np.concatenate((Xerr_list, [Xerr]), axis=0)  # Append Xerr to Xerr_list

                wheel_velocities = np.array(controls[0:4])  # Extract wheel velocities from controls
                joint_velocities = np.array(controls[4:])  # Extract joint velocities from controls
                combined_velocities = np.concatenate((joint_velocities, wheel_velocities))  # Combine joint and wheel velocities

                if not isinstance(initial_config, RobotConfiguration):  # Check if initial_config is an instance of RobotConfiguration
                    raise TypeError("initial_config is not an instance of RobotConfiguration")

                final_configuration = NextState(initial_config, combined_velocities, joint_speed_limit)  # Calculate the next state
                initial_config = final_configuration  # Update initial_config for the next iteration

            # Collect data rows for all segments
            all_configuration_data_rows.extend(configuration_data_rows)  # Add configuration data rows to the main list

        # Write data to a file
        if task == "best":
            write_to_csv(all_configuration_data_rows, fn='result_best.csv')  # Write all data rows to a CSV file
        elif task == "overshoot":
            write_to_csv(all_configuration_data_rows, fn='result_overshoot.csv')
        elif task == "newtask":
            write_to_csv(all_configuration_data_rows, fn='result_newtask.csv')    

        print("Producing error Plot")  # Print message
        print("P: " + str(Kp[0, 0]) + " I: " + str(Ki))  # Print Kp and Ki values    
        store_xerr(Xerr_list)  # Store Xerr values
        Xerr_plot(Xerr_list, Kp, Ki)  # Plot Xerr values

        print("Script Completed")  # Print completion message
    except Exception as e:
        print(f"Error in main script: {e}")
        sys.exit(1)
