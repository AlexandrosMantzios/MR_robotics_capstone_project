"""
30-05-2024
Capstone Project - Modern robotics
Trajectories.py : Module to compute the trajectory segments between T_initial 
and T_goal. Creates trajectory and stores values. 

@ Alexandros Mantzios

"""

# import csv
# import os
# import numpy as np
# import math as m
# import modern_robotics as mr
# import math
# from Configurations import RobotConfiguration, Parameters, Parameters, Cube_Task, RobotProperties
# from scipy.spatial.transform import Rotation as Rot


# time_scale = Parameters.time_scale
# trajectory_type = Parameters.trajectory_type
# # Initial cube configuration 
# Tsc_init = Cube_Task.Tsc_init()
# cube_task = Cube_Task
# # Initial EE robot configuration 
# Tse_init = RobotProperties.Tse_init()
# # Goal cube task configuration 
# Tsc_goal = Cube_Task.Tsc_goal()
# # Pre_pick EE configuration 
# Tce_stand = Cube_Task.Tce_standoff()
# # Pick EE configuration 
# Tce_grip = Cube_Task.Tce_grip()
# # k parameter 
# k = Parameters.k()
# max_linear_velocity = Parameters.max_linear_velocity
# max_angular_velocity = Parameters.max_angular_velocity


# class TrajectoryPlanner():

#     """
#     Class Trajectory planner to contain methods for calculating trajectories
#     """

#     def gen_data_row(conf_mat, grip_state):  
#         """
#         Function to generate the rows of the csv files
#         Inputs : 
#         - Configuration matrix
#         - Gripper state
#         Outputs : 
#         - data row -> combination of conf and gripper state 
#         """
#         rot = conf_mat[:-1, 0:-1].flatten()  
#         pos = conf_mat[:-1, -1:].flatten()   

#         data_row = np.concatenate((rot, pos, grip_state), axis=None)  
#         data_row = data_row.tolist()  
#         return data_row  

#     def write_csv_row(conf_mat, grip_state):  
#         """
#         Function to write conf to csv 
#         Inputs:
#         - configuration
#         - gripper state (boolean)
#         """
#         data_row = TrajectoryPlanner.gen_data_row(conf_mat, grip_state)  
#         # Get the current directory
#         current_dir = os.getcwd()
#         folder_name = "test_alex"
#         folder_path = os.path.join(current_dir, folder_name)

#         # Create the folder if it doesn't exist
#         if not os.path.exists(folder_path):
#             os.makedirs(folder_path)
#             print(f"Folder '{folder_name}' created at: {folder_path}")
#         filename = os.path.join(folder_path, 'trajectory_new_test.csv')  # Join the folder path with the filename
#         print(f"Writing data to file: {filename}")

#         with open(filename, 'a', newline='') as f:  
#             csv_writer = csv.writer(f)  
#             csv_writer.writerow(data_row)  
#             print("Data written to file successfully!")

#         # Create the folder if it doesn't exist
#         if not os.path.exists(folder_path):
#             os.makedirs(folder_path)

#         filename = os.path.join(folder_path, 'trajectory_new_test.csv')  # Join the folder path with the filename

#         with open(filename, 'a', newline='') as f:  
#             csv_writer = csv.writer(f)  
#             csv_writer.writerow(data_row)  

    
#     def write_trajectory_to_csv(traj_list,gripper_state):
#         """
#         Function to write trajectory to csv
#         """
#         for conf in traj_list:
#           TrajectoryPlanner.gen_data_row(conf,gripper_state)


#     def gripper_status(configuration,k):
#         """
#         Function to get gripper status and add it to trajectory list
#         Inputs:
#         - configuration
#         - k (1 but can differ)
#         Outputs : 
#         - Trajectory list with number of confs
#         """
#         # 0.5 seconds to close the gripper
#         Tf = 0.5 
#         # Integer number of configurations
#         N = int(Tf * k/0.01)
#         # List of trajectories based on configuration numbers
#         trajectory_list = [configuration]*N
#         return trajectory_list

#     def trajectory_generator(pos1, pos2):
#         """
#         Function to calculate Euclidian distance between two positions
#         Inputs: 
#         - position1, position2
#         Outputs:
#         - Distance between positions
#         """
#         # Euclidian Calculate distance
#         distance = np.linalg.norm(pos1 - pos2)
#         return distance

#     def extract_position_vector(frame):
#         """
#         Function to extract position vector 
#         """
#         return frame[:-1,-1:].flatten()

#     def calculate_trajectory_parameters(pos_start, pos_end, max_velocity, k):
#         """
#         Function to calculate trajectory parameters
#         Inputs:
#         - Start position
#         - Goal position
#         - maximum velocity 
#         - k 
#         Outputs:
#         - Duration of trajectory 
#         - Number of configurations
#         """
#         pos1 = TrajectoryPlanner.extract_position_vector(pos_start)
#         pos2 = TrajectoryPlanner.extract_position_vector(pos_end)
    
#         d = TrajectoryPlanner.trajectory_generator(pos1, pos2)
#         Tf = round(d/max_velocity,3)
#         No_conf = Tf*k/0.01
#         return Tf,No_conf

#     def Trajectory(T_ini, T_goal, max_v, k, trajectory_type):  
#         """
#         Function to get trajectory list
#         Inputs: 
#         - Initial configuration
#         - End configuration
#         - Maximum velocity
#         - k
#         - trajectory type
#         Output:
#         - Trajectory list
#         """

#         Tf,No_conf = TrajectoryPlanner.calculate_trajectory_parameters(T_ini, T_goal, max_v,k)
#         # Compute difference between initial and goal configurations
#         T_diff = np.dot(mr.TransInv(T_ini), T_goal) 
#         # Compute angles change from rotation matrix
#         angles_change = max(TrajectoryPlanner.rotation_matrix_to_euler_angles(T_diff[:-1, :-1])) 
#         # Compute linear distance to cover
#         Distance = math.hypot(T_diff[0, -1], T_diff[1, -1])
#         # Compute duration of move 
#         Duration = max(Distance / max_linear_velocity, angles_change / max_angular_velocity)
#         No_config = int((Duration*k)/0.01)
#         if trajectory_type == "CartesianTrajectory":
#             trajectory_list = mr.CartesianTrajectory(T_ini, T_goal, Tf, No_conf, time_scale)
#         elif trajectory_type == "ScrewTrajectory":
#             trajectory_list = mr.ScrewTrajectory(T_ini,T_goal, Tf, No_conf, time_scale)
#         return trajectory_list
    
#     def rotation_matrix_to_euler_angles(Rot_matrix):
#             """
#             Convert rotation matrix to Euler angles (alpha, beta, gamma).
#             Inputs:
#             - Rot_matrix: Input rotation matrix
#             Output:
#             - Euler angles [alpha, beta, gamma]
#             """
#             # Compute the beta angle from Rotation matrix
#             beta = np.arcsin(-np.clip(Rot_matrix[2, 0], -1.0, 1.0)) 
#             # Check singularity avoidance condition 
#             if np.abs(Rot_matrix[2, 0]) < 0.99999:  
#                 # Compute alpha angle
#                 alpha = np.arctan2(Rot_matrix[1, 0], Rot_matrix[0, 0])  
#                 # Compute gamma angle
#                 gamma = np.arctan2(Rot_matrix[2, 1], Rot_matrix[2, 2])   
#                 # print("alpha, gamma", alpha, gamma)  
#             else:
#                 # if singularity avoidance condition not met 
#                 # set alpha angle = 0
#                 alpha = 0.0  
#                 # Compute gamma angle
#                 gamma = np.arctan2(-Rot_matrix[0, 1], Rot_matrix[1, 1])  
#                 # print("alpha, gamma", alpha, gamma)  
#             # returns euler angles as numpy array
#             return np.array([alpha, beta, gamma], dtype=float)
    
#     @staticmethod
#     def pre_processing(Tsc_init, Tsc_goal, Tse_init, max_velocity, k):

#         Tse_init_standoff = np.dot(Tsc_init, Tce_stand)
#         Tse_init_grip = np.dot(Tsc_init, Tce_grip)
#         Tse_goal_standoff = np.dot(Tsc_goal, Tce_stand)
#         Tse_goal_grip = np.dot(Tsc_goal, Tce_grip)

#         return Tse_init_standoff, Tse_init_grip ,Tse_goal_standoff ,Tse_goal_grip



#     def Full_trajectory(Tsc_init, Tsc_goal, Tse_init, k, max_velocity):
#         """
#         Function to get full trajectory comprising of all segments
#         Inputs:
#         - Tsc initial 
#         - Tsc goal 
#         - Tse initial
#         - k
#         - Maximum velocity
#         Ouput: 
#         - Full trajectory list 12 vector
#         - Full gripper 1 vector
#         """

#         Full_trajectory_list = []
#         Full_gripper_state = []
#         rob_prop = RobotProperties
#         trajectory_type = Parameters.trajectory_type
#         # Tse from Modern Robotics Wiki Page
#         Tse_init = np.array([[0,0,1,0],
#                       [0,1,0,0],
#                       [-1,0,0,0.5],
#                       [0,0,0,1]])
#         #Tse_init_standoff, Tse_init_grip, Tse_goal_standoff, Tse_goal_grip = TrajectoryPlanner.pre_processing(Tsc_init, Tsc_goal, Tse_init,max_velocity,k)
#         Tse_init_standoff, Tse_init_grip, Tse_goal_standoff, Tse_goal_grip = TrajectoryPlanner.pre_processing(Tsc_init, Tsc_goal, Tse_init, max_velocity, k)


#         # Initial configuration to Tse_pre-grip 
#         T_init = Tse_init
#         T_goal = Tse_init_standoff
#         Trajectory_list = TrajectoryPlanner.Trajectory(T_init, T_goal, max_velocity, k, trajectory_type)
#         Full_trajectory_list.append(Trajectory_list)
#         gripper_state = 0
#         Full_gripper_state.append(gripper_state)
#         # Move to grip configuration 
#         T_init = T_goal
#         T_goal = Tse_init_grip
#         Trajectory_list = TrajectoryPlanner.Trajectory(T_init, T_goal, max_velocity, k,trajectory_type)
#         Full_trajectory_list.append(Trajectory_list)
#         gripper_state = 0 
#         Full_gripper_state.append(gripper_state)
#         # Close gripper 
#         Trajectory_list = TrajectoryPlanner.gripper_status(T_goal, k)
#         Full_trajectory_list.append(Trajectory_list)
#         gripper_state = 1 
#         Full_gripper_state.append(gripper_state)
#         # Move back  to Tse_pre-grip 
#         T_init = T_goal
#         T_goal = Tse_init_standoff
#         Trajectory_list = TrajectoryPlanner.Trajectory(T_init, T_goal, max_velocity, k,trajectory_type)
#         Full_trajectory_list.append(Trajectory_list)
#         gripper_state = 1
#         Full_gripper_state.append(gripper_state)
#         # Move from current conf to desired pre-grip 
#         T_init = T_goal
#         T_goal = Tse_goal_standoff
#         Trajectory_list = TrajectoryPlanner.Trajectory(T_init, T_goal, max_velocity, k,trajectory_type)
#         Full_trajectory_list.append(Trajectory_list)
#         gripper_state=1 
#         Full_gripper_state.append(gripper_state)
#         # Move to place position 
#         T_init = T_goal
#         T_goal = Tse_goal_grip
#         Trajectory_list = TrajectoryPlanner.Trajectory(T_init, T_goal, max_velocity, k,trajectory_type)
#         Full_trajectory_list.append(Trajectory_list)
#         gripper_state=1 
#         Full_gripper_state.append(gripper_state)   
#         # Open gripper 
#         Trajectory_list = TrajectoryPlanner.gripper_status(T_goal, k)
#         Full_trajectory_list.append(Trajectory_list)
#         gripper_state = 0 
#         Full_gripper_state.append(gripper_state)
#         # Move from current conf to desired pre-grip 
#         T_init = T_goal
#         T_goal = Tse_goal_standoff
#         Trajectory_list = TrajectoryPlanner.Trajectory(T_init, T_goal, max_velocity, k,trajectory_type)
#         Full_trajectory_list.append(Trajectory_list)
#         gripper_state=0 
#         Full_gripper_state.append(gripper_state)

#         return Full_trajectory_list, Full_gripper_state


##############################################################################
# 2nd version 
    

    # Import necessary modules and classes
import csv
import os
import numpy as np
import math as m
import core as mr
import math
from Configurations import RobotConfiguration, Parameters, Cube_Task, RobotProperties
from scipy.spatial.transform import Rotation as Rot

# Set parameters from imported modules
time_scale = Parameters.time_scale
trajectory_type = Parameters.trajectory_type
cube_task = Cube_Task
# Initial cube configuration - best and overshoot
Tsc_init = Cube_Task.Tsc_init()

# # Initial cube configuration - newtask
# Tsc_init = Cube_Task.Tsc_init_new()


# Initial end effector (EE) robot configuration 
Tse_init = RobotProperties.Tse_init()

# Goal cube task configuration - Uncomment for best and overshoot tasks
Tsc_goal = Cube_Task.Tsc_goal()

# # Goal cube task configuration - Uncomment for newtask task 
# Tsc_goal = Cube_Task.Tsc_goal_new()

# Pre-pick EE configuration 
Tce_stand = Cube_Task.Tce_standoff()

# Pick EE configuration 
Tce_grip = Cube_Task.Tce_grip()

# k parameter 
k = Parameters.k()

# Maximum velocities
max_linear_velocity = Parameters.max_linear_velocity
max_angular_velocity = Parameters.max_angular_velocity

class TrajectoryPlanner():
    """
    Class TrajectoryPlanner to contain methods for calculating trajectories
    """

    def gen_data_row(conf_mat, grip_state):  
        """
        Function to generate the rows of the csv files
        Inputs : 
        - conf_mat : Configuration matrix
        - grip_state : Gripper state
        Outputs : 
        - data_row : Combination of configuration and gripper state 
        """
        rot = conf_mat[:-1, 0:-1].flatten()  # Flatten the rotation part of the configuration matrix
        pos = conf_mat[:-1, -1:].flatten()   # Flatten the position part of the configuration matrix

        data_row = np.concatenate((rot, pos, grip_state), axis=None)  # Concatenate rotation, position, and gripper state
        data_row = data_row.tolist()  # Convert the data row to a list
        return data_row  # Return the data row

    def write_csv_row(conf_mat, grip_state):  
        """
        Function to write configuration to CSV 
        Inputs:
        - conf_mat : Configuration matrix
        - grip_state : Gripper state (boolean)
        """
        data_row = TrajectoryPlanner.gen_data_row(conf_mat, grip_state)  # Generate data row
        current_dir = os.getcwd()  # Get the current directory
        folder_name = "test_alex"  # Define the folder name
        folder_path = os.path.join(current_dir, folder_name)  # Create the folder path

        # Create the folder if it doesn't exist
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
            print(f"Folder '{folder_name}' created at: {folder_path}")
        
        filename = os.path.join(folder_path, 'trajectory_new_test.csv')  # Create the filename path
        print(f"Writing data to file: {filename}")

        with open(filename, 'a', newline='') as f:  # Open the CSV file in append mode
            csv_writer = csv.writer(f)  # Create a CSV writer object
            csv_writer.writerow(data_row)  # Write the data row to the CSV file
            print("Data written to file successfully!")

    def write_trajectory_to_csv(traj_list, gripper_state):
        """
        Function to write trajectory to CSV
        Inputs:
        - traj_list : List of trajectory configurations
        - gripper_state : Gripper state
        """
        for conf in traj_list:  # Iterate through the trajectory list
            TrajectoryPlanner.write_csv_row(conf, gripper_state)  # Write each configuration to CSV

    def gripper_status(configuration, k):
        """
        Function to get gripper status and add it to trajectory list
        Inputs:
        - configuration : Configuration matrix
        - k : Scaling factor
        Outputs : 
        - trajectory_list : List of configurations with gripper status
        """
        Tf = 0.5  # 0.5 seconds to close the gripper
        N = int(Tf * k / 0.01)  # Calculate the number of configurations
        trajectory_list = [configuration] * N  # Create a list of configurations
        return trajectory_list  # Return the trajectory list

    def trajectory_generator(pos1, pos2):
        """
        Function to calculate Euclidean distance between two positions
        Inputs: 
        - pos1 : Start position
        - pos2 : End position
        Outputs:
        - distance : Distance between positions
        """
        distance = np.linalg.norm(pos1 - pos2)  # Calculate Euclidean distance
        return distance  # Return the distance

    def extract_position_vector(frame):
        """
        Function to extract position vector 
        Inputs:
        - frame : Homogeneous transformation matrix
        Outputs:
        - position_vector : Extracted position vector
        """
        return frame[:-1, -1:].flatten()  # Extract and flatten the position vector

    def calculate_trajectory_parameters(pos_start, pos_end, max_velocity, k):
        """
        Function to calculate trajectory parameters
        Inputs:
        - pos_start : Start position
        - pos_end : Goal position
        - max_velocity : Maximum velocity
        - k : Scaling factor
        Outputs:
        - Tf : Duration of trajectory
        - No_conf : Number of configurations
        """
        pos1 = TrajectoryPlanner.extract_position_vector(pos_start)  # Extract start position vector
        pos2 = TrajectoryPlanner.extract_position_vector(pos_end)  # Extract end position vector

        d = TrajectoryPlanner.trajectory_generator(pos1, pos2)  # Calculate distance
        Tf = round(d / max_velocity, 3)  # Calculate duration of trajectory
        No_conf = Tf * k / 0.01  # Calculate number of configurations
        return Tf, No_conf  # Return duration and number of configurations

    def Trajectory(T_ini, T_goal, max_v, k, trajectory_type):  
        """
        Function to get trajectory list
        Inputs: 
        - T_ini : Initial configuration
        - T_goal : End configuration
        - max_v : Maximum velocity
        - k : Scaling factor
        - trajectory_type : Type of trajectory
        Outputs:
        - trajectory_list : List of configurations in the trajectory
        """
        Tf, No_conf = TrajectoryPlanner.calculate_trajectory_parameters(T_ini, T_goal, max_v, k)  # Calculate trajectory parameters

        T_diff = np.dot(mr.TransInv(T_ini), T_goal)  # Compute difference between initial and goal configurations
        angles_change = max(TrajectoryPlanner.rotation_matrix_to_euler_angles(T_diff[:-1, :-1]))  # Compute angles change
        Distance = math.hypot(T_diff[0, -1], T_diff[1, -1])  # Compute linear distance to cover
        Duration = max(Distance / max_linear_velocity, angles_change / max_angular_velocity)  # Compute duration of move
        No_config = int((Duration * k) / 0.01)  # Calculate number of configurations

        # Generate trajectory based on type
        if trajectory_type == "CartesianTrajectory":
            trajectory_list = mr.CartesianTrajectory(T_ini, T_goal, Tf, No_conf, time_scale)
        elif trajectory_type == "ScrewTrajectory":
            trajectory_list = mr.ScrewTrajectory(T_ini, T_goal, Tf, No_conf, time_scale)
        return trajectory_list  # Return the trajectory list

    def rotation_matrix_to_euler_angles(Rot_matrix):
        """
        Convert rotation matrix to Euler angles (alpha, beta, gamma).
        Inputs:
        - Rot_matrix : Input rotation matrix
        Outputs:
        - Euler angles [alpha, beta, gamma]
        """
        beta = np.arcsin(-np.clip(Rot_matrix[2, 0], -1.0, 1.0))  # Compute the beta angle
        if np.abs(Rot_matrix[2, 0]) < 0.99999:  # Check singularity avoidance condition 
            alpha = np.arctan2(Rot_matrix[1, 0], Rot_matrix[0, 0])  # Compute alpha angle
            gamma = np.arctan2(Rot_matrix[2, 1], Rot_matrix[2, 2])   # Compute gamma angle
        else:
            alpha = 0.0  # Set alpha angle to 0
            gamma = np.arctan2(-Rot_matrix[0, 1], Rot_matrix[1, 1])  # Compute gamma angle
        return np.array([alpha, beta, gamma], dtype=float)  # Return Euler angles as numpy array
    
    @staticmethod
    def pre_processing(Tsc_init, Tsc_goal, Tse_init, max_velocity, k):
        """
        Pre-processing step to calculate initial and goal configurations
        Inputs:
        - Tsc_init : Initial cube configuration
        - Tsc_goal : Goal cube configuration
        - Tse_init : Initial end effector configuration
        - max_velocity : Maximum velocity
        - k : Scaling factor
        Outputs:
        - Tse_init_standoff : Pre-pick EE configuration
        - Tse_init_grip : Pick EE configuration
        - Tse_goal_standoff : Pre-place EE configuration
        - Tse_goal_grip : Place EE configuration
        """
        Tse_init_standoff = np.dot(Tsc_init, Tce_stand)  # Calculate initial standoff configuration
        Tse_init_grip = np.dot(Tsc_init, Tce_grip)  # Calculate initial grip configuration
        Tse_goal_standoff = np.dot(Tsc_goal, Tce_stand)  # Calculate goal standoff configuration
        Tse_goal_grip = np.dot(Tsc_goal, Tce_grip)  # Calculate goal grip configuration

        return Tse_init_standoff, Tse_init_grip, Tse_goal_standoff, Tse_goal_grip  # Return configurations

    def Full_trajectory(Tsc_init, Tsc_goal, Tse_init, k, max_velocity):
        """
        Function to get full trajectory comprising of all segments
        Inputs:
        - Tsc_init : Initial cube configuration
        - Tsc_goal : Goal cube configuration
        - Tse_init : Initial end effector configuration
        - k : Scaling factor
        - max_velocity : Maximum velocity
        Outputs:
        - Full_trajectory_list : List of all configurations
        - Full_gripper_state : List of gripper states
        """
        Full_trajectory_list = []  # Initialize full trajectory list
        Full_gripper_state = []  # Initialize full gripper state list
        rob_prop = RobotProperties  # Alias for RobotProperties
        trajectory_type = Parameters.trajectory_type  # Get trajectory type

        # Define initial end effector configuration
        Tse_init = np.array([[0,0,1,0],
                             [0,1,0,0],
                             [-1,0,0,0.5],
                             [0,0,0,1]])
        # Calculate necessary configurations
        Tse_init_standoff, Tse_init_grip, Tse_goal_standoff, Tse_goal_grip = TrajectoryPlanner.pre_processing(Tsc_init, Tsc_goal, Tse_init, max_velocity, k)

        # Add initial to pre-grip trajectory
        T_init = Tse_init
        T_goal = Tse_init_standoff
        Trajectory_list = TrajectoryPlanner.Trajectory(T_init, T_goal, max_velocity, k, trajectory_type)
        Full_trajectory_list.append(Trajectory_list)
        gripper_state = 0  # Gripper open
        Full_gripper_state.append(gripper_state)

        # Add move to grip configuration
        T_init = T_goal
        T_goal = Tse_init_grip
        Trajectory_list = TrajectoryPlanner.Trajectory(T_init, T_goal, max_velocity, k, trajectory_type)
        Full_trajectory_list.append(Trajectory_list)
        gripper_state = 0  # Gripper open
        Full_gripper_state.append(gripper_state)

        # Add close gripper configuration
        Trajectory_list = TrajectoryPlanner.gripper_status(T_goal, k)
        Full_trajectory_list.append(Trajectory_list)
        gripper_state = 1  # Gripper closed
        Full_gripper_state.append(gripper_state)

        # Add move back to pre-grip configuration
        T_init = T_goal
        T_goal = Tse_init_standoff
        Trajectory_list = TrajectoryPlanner.Trajectory(T_init, T_goal, max_velocity, k, trajectory_type)
        Full_trajectory_list.append(Trajectory_list)
        gripper_state = 1  # Gripper closed
        Full_gripper_state.append(gripper_state)

        # Add move to desired pre-place configuration
        T_init = T_goal
        T_goal = Tse_goal_standoff
        Trajectory_list = TrajectoryPlanner.Trajectory(T_init, T_goal, max_velocity, k, trajectory_type)
        Full_trajectory_list.append(Trajectory_list)
        gripper_state = 1  # Gripper closed
        Full_gripper_state.append(gripper_state)

        # Add move to place position
        T_init = T_goal
        T_goal = Tse_goal_grip
        Trajectory_list = TrajectoryPlanner.Trajectory(T_init, T_goal, max_velocity, k, trajectory_type)
        Full_trajectory_list.append(Trajectory_list)
        gripper_state = 1  # Gripper closed
        Full_gripper_state.append(gripper_state)

        # Add open gripper configuration
        Trajectory_list = TrajectoryPlanner.gripper_status(T_goal, k)
        Full_trajectory_list.append(Trajectory_list)
        gripper_state = 0  # Gripper open
        Full_gripper_state.append(gripper_state)

        # Add move to desired pre-place configuration
        T_init = T_goal
        T_goal = Tse_goal_standoff
        Trajectory_list = TrajectoryPlanner.Trajectory(T_init, T_goal, max_velocity, k, trajectory_type)
        Full_trajectory_list.append(Trajectory_list)
        gripper_state = 0  # Gripper open
        Full_gripper_state.append(gripper_state)

        return Full_trajectory_list, Full_gripper_state  # Return full trajectory and gripper state lists
    



    








        

    





  