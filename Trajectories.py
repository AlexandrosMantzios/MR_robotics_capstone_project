"""
30-05-2024
Capstone Project - Modern robotics
Trajectories.py : Module to compute the trajectory segments between T_initial
and T_goal. Creates trajectory and stores values.

@ Alexandros Mantzios
"""

import numpy as np
import math
import core as mr
from Configurations import Parameters


# Time scaling for trajectory polynomials (Cubic=3, Quintic=5)
time_scale = Parameters.time_scale


class TrajectoryPlanner():
    """
    Class TrajectoryPlanner to contain methods for calculating trajectories.
    Generates 8-segment pick-and-place trajectories.
    """

    @staticmethod
    def gripper_status(configuration, k):
        """
        Function to get gripper status and add it to trajectory list.
        Holds the given configuration for 0.5 seconds.
        Inputs:
        - configuration : Configuration matrix (4x4 SE(3))
        - k : Scaling factor
        Outputs :
        - trajectory_list : List of repeated configurations
        """
        Tf = 0.5  # 0.5 seconds to close/open the gripper
        N = int(Tf * k / 0.01)  # Calculate the number of configurations
        trajectory_list = [configuration] * N
        return trajectory_list

    @staticmethod
    def rotation_matrix_to_euler_angles(Rot_matrix):
        """
        Convert rotation matrix to Euler angles (alpha, beta, gamma).
        Inputs:
        - Rot_matrix : 3x3 rotation matrix
        Outputs:
        - Euler angles [alpha, beta, gamma] as numpy array
        """
        beta = np.arcsin(-np.clip(Rot_matrix[2, 0], -1.0, 1.0))  # Compute beta angle
        if np.abs(Rot_matrix[2, 0]) < 0.99999:  # Singularity avoidance
            alpha = np.arctan2(Rot_matrix[1, 0], Rot_matrix[0, 0])
            gamma = np.arctan2(Rot_matrix[2, 1], Rot_matrix[2, 2])
        else:
            alpha = 0.0
            gamma = np.arctan2(-Rot_matrix[0, 1], Rot_matrix[1, 1])
        return np.array([alpha, beta, gamma], dtype=float)

    @staticmethod
    def Trajectory(T_ini, T_goal, max_v, max_omega, k, trajectory_type):
        """
        Function to get trajectory list between two SE(3) poses.
        Duration is determined by whichever is slower: linear or angular motion.
        Inputs:
        - T_ini : Initial configuration (4x4 SE(3))
        - T_goal : End configuration (4x4 SE(3))
        - max_v : Maximum linear velocity
        - max_omega : Maximum angular velocity
        - k : Scaling factor
        - trajectory_type : "CartesianTrajectory" or "ScrewTrajectory"
        Outputs:
        - trajectory_list : List of 4x4 SE(3) configurations
        """
        # Compute relative transform
        T_diff = np.dot(mr.TransInv(T_ini), T_goal)
        # Compute angular change
        angles_change = np.max(np.abs(TrajectoryPlanner.rotation_matrix_to_euler_angles(T_diff[:-1, :-1])))
        # Compute linear distance
        Distance = math.hypot(T_diff[0, -1], T_diff[1, -1])
        # Duration is max of linear and angular time, minimum 0.01s
        Tf = max(Distance / max_v, angles_change / max_omega, 0.01)
        # Number of configurations, minimum 2
        No_conf = max(int((Tf * k) / 0.01), 2)

        # Generate trajectory based on type
        if trajectory_type == "CartesianTrajectory":
            trajectory_list = mr.CartesianTrajectory(T_ini, T_goal, Tf, No_conf, time_scale)
        elif trajectory_type == "ScrewTrajectory":
            trajectory_list = mr.ScrewTrajectory(T_ini, T_goal, Tf, No_conf, time_scale)
        return trajectory_list

    @staticmethod
    def pre_processing(Tsc_init, Tsc_goal, Tce_stand, Tce_grip):
        """
        Pre-processing step to calculate intermediate waypoint poses
        by combining cube poses with gripper offsets.
        Inputs:
        - Tsc_init : Initial cube configuration in {s}
        - Tsc_goal : Goal cube configuration in {s}
        - Tce_stand : EE standoff pose relative to cube
        - Tce_grip : EE grip pose relative to cube
        Outputs:
        - Tse_init_standoff : Pre-pick EE configuration (hover above pick)
        - Tse_init_grip : Pick EE configuration (at cube)
        - Tse_goal_standoff : Pre-place EE configuration (hover above place)
        - Tse_goal_grip : Place EE configuration (at goal)
        """
        Tse_init_standoff = np.dot(Tsc_init, Tce_stand)
        Tse_init_grip = np.dot(Tsc_init, Tce_grip)
        Tse_goal_standoff = np.dot(Tsc_goal, Tce_stand)
        Tse_goal_grip = np.dot(Tsc_goal, Tce_grip)
        return Tse_init_standoff, Tse_init_grip, Tse_goal_standoff, Tse_goal_grip

    @staticmethod
    def Full_trajectory(Tsc_init, Tsc_goal, Tse_init, Tce_stand, Tce_grip,
                        k, max_velocity, max_angular_velocity):
        """
        Function to get full trajectory comprising of all 8 segments:
        1. Home → pre-pick standoff (gripper open)
        2. Pre-pick standoff → grip pose (gripper open)
        3. Close gripper (hold 0.5s)
        4. Grip pose → pre-pick standoff (gripper closed)
        5. Pre-pick standoff → pre-place standoff (gripper closed)
        6. Pre-place standoff → place pose (gripper closed)
        7. Open gripper (hold 0.5s)
        8. Place pose → pre-place standoff (gripper open)

        Inputs:
        - Tsc_init : Initial cube configuration in {s}
        - Tsc_goal : Goal cube configuration in {s}
        - Tse_init : Initial end effector configuration in {s}
        - Tce_stand : EE standoff pose relative to cube
        - Tce_grip : EE grip pose relative to cube
        - k : Scaling factor
        - max_velocity : Maximum linear velocity
        - max_angular_velocity : Maximum angular velocity
        Outputs:
        - Full_trajectory_list : List of 8 trajectory segments
        - Full_gripper_state : List of 8 gripper states (0=open, 1=closed)
        """
        Full_trajectory_list = []
        Full_gripper_state = []
        trajectory_type = Parameters.trajectory_type
        max_v = max_velocity
        max_omega = max_angular_velocity

        # Calculate necessary waypoint configurations
        Tse_init_standoff, Tse_init_grip, Tse_goal_standoff, Tse_goal_grip = \
            TrajectoryPlanner.pre_processing(Tsc_init, Tsc_goal, Tce_stand, Tce_grip)

        # Segment 1: Home → pre-pick standoff (gripper open)
        T_init = Tse_init
        T_goal = Tse_init_standoff
        Trajectory_list = TrajectoryPlanner.Trajectory(T_init, T_goal, max_v, max_omega, k, trajectory_type)
        Full_trajectory_list.append(Trajectory_list)
        Full_gripper_state.append(0)  # Gripper open

        # Segment 2: Move to grip configuration (gripper open)
        T_init = T_goal
        T_goal = Tse_init_grip
        Trajectory_list = TrajectoryPlanner.Trajectory(T_init, T_goal, max_v, max_omega, k, trajectory_type)
        Full_trajectory_list.append(Trajectory_list)
        Full_gripper_state.append(0)  # Gripper open

        # Segment 3: Close gripper (hold position)
        Trajectory_list = TrajectoryPlanner.gripper_status(T_goal, k)
        Full_trajectory_list.append(Trajectory_list)
        Full_gripper_state.append(1)  # Gripper closed

        # Segment 4: Move back to pre-pick standoff (gripper closed)
        T_init = T_goal
        T_goal = Tse_init_standoff
        Trajectory_list = TrajectoryPlanner.Trajectory(T_init, T_goal, max_v, max_omega, k, trajectory_type)
        Full_trajectory_list.append(Trajectory_list)
        Full_gripper_state.append(1)  # Gripper closed

        # Segment 5: Move to pre-place standoff (gripper closed)
        T_init = T_goal
        T_goal = Tse_goal_standoff
        Trajectory_list = TrajectoryPlanner.Trajectory(T_init, T_goal, max_v, max_omega, k, trajectory_type)
        Full_trajectory_list.append(Trajectory_list)
        Full_gripper_state.append(1)  # Gripper closed

        # Segment 6: Move to place position (gripper closed)
        T_init = T_goal
        T_goal = Tse_goal_grip
        Trajectory_list = TrajectoryPlanner.Trajectory(T_init, T_goal, max_v, max_omega, k, trajectory_type)
        Full_trajectory_list.append(Trajectory_list)
        Full_gripper_state.append(1)  # Gripper closed

        # Segment 7: Open gripper (hold position)
        Trajectory_list = TrajectoryPlanner.gripper_status(T_goal, k)
        Full_trajectory_list.append(Trajectory_list)
        Full_gripper_state.append(0)  # Gripper open

        # Segment 8: Move back to pre-place standoff (gripper open)
        T_init = T_goal
        T_goal = Tse_goal_standoff
        Trajectory_list = TrajectoryPlanner.Trajectory(T_init, T_goal, max_v, max_omega, k, trajectory_type)
        Full_trajectory_list.append(Trajectory_list)
        Full_gripper_state.append(0)  # Gripper open

        return Full_trajectory_list, Full_gripper_state
