"""
04-05-2024
Capstone Project - Modern robotics
FeedbackController.py : Uses reference trajectory and calculates the joint 
and wheel velocities to get robot to desired configuration

@ Alexandros Mantzios

"""

import matplotlib.pyplot as plt
import sys
import os
import numpy as np
import modern_robotics as mr
from math import cos, sin, fabs, copysign
from Configurations import RobotConfiguration, RobotProperties, Cube_Task, Parameters
from numpy.linalg import inv

# Robot properties and initial configuration
try:
    # Initializing base configuration of robot
    Tb0 = RobotProperties.Tb0()
    # Initializing home configuration of robot
    M_Home = RobotProperties.M_Home()
    # Initializing screw axes of the robot
    Blist = RobotProperties.Blist()
    # Initializing integral part for PID control
    integral_part = np.zeros(6)
    # Create instance of RobotProperties class
    robot_properties = RobotProperties()
    # Assigning wheel radius
    r = robot_properties.r
    # Assigning distance to wheel axis
    l = robot_properties.l
    # Assigning half wheel axis
    ω = robot_properties.ω
    # Assigning height of the base
    height = robot_properties.height
except AttributeError as e:
    print(f"Error in initializing robot properties: {e}", file=sys.stderr)
    sys.exit(1)

# Setting print options for numpy arrays
try:
    # Suppress scientific notation
    np.set_printoptions(suppress=True)
    # Set precision to 4 decimal places
    np.set_printoptions(precision=4)
except Exception as e:
    print(f"Error in setting numpy print options: {e}", file=sys.stderr)

# Defining the Controller class
class Controller:
    """Class for Feedback Controller"""

    @staticmethod
    def Jacobian_robot(configuration, Joint_Limit_test):
        try:
            """
            Jacobian for mobile robot base 
            Inputs : 
            - configuration : 13 element vector 
            - Joint_Limit_test result (True or False)
            Outputs :
            - J_robot -> Jacobian for entire robot
            """
            chassi_config = configuration.get_chassi_configuration()  # Get chassis configuration
            joint_angles = configuration.get_joint_angles()  # Get joint angles
            Tb0 = RobotProperties.Tb0()  # Get base configuration
            M_Home = RobotProperties.M_Home()  # Get home configuration
            Blist = RobotProperties.Blist()  # Get screw axes
            robot_properties = RobotProperties()  # Create an instance of RobotProperties
            r = robot_properties.r  # Assign wheel radius
            l = robot_properties.l  # Assign distance to wheel axis
            ω = robot_properties.ω  # Assign half wheel axis
            height = robot_properties.height  # Assign height of the base

            φ = chassi_config[0]  # Extract phi from chassis configuration
            x = chassi_config[1]  # Extract x from chassis configuration
            y = chassi_config[2]  # Extract y from chassis configuration

            # Chassis base with respect to space frame
            Tsb = np.array([[cos(φ), -sin(φ), 0, x],
                            [sin(φ), cos(φ), 0, y],
                            [0, 0, 1, height],
                            [0, 0, 0, 1]])
            
            # Robot arm base with respect to end effector
            T0e = mr.FKinBody(M_Home, Blist.T, joint_angles)  # Calculate forward kinematics
            Adj_Teb = mr.Adjoint(mr.TransInv(T0e))  # Calculate adjoint transformation

            # Calculate wheel Jacobian
            F = (r/4) * np.array([[-1/(l + ω), 1/(l + ω), 1/(l + ω), -1/(l + ω)],
                                  [1, 1, 1, 1],
                                  [-1, 1, -1, 1]])

            # Create a 6x4 zero matrix for full Jacobian
            F6 = np.zeros((6, 4))
            F6[2:-1, :] = F  # Assign wheel Jacobian values

            # Calculate adjoint transformation between base and end effector
            Teb = np.dot(Controller.pinv_tol(T0e), Controller.pinv_tol(Tb0))
            J_base = np.dot(mr.Adjoint(Teb), F6)  # Calculate base Jacobian

            J_arm = mr.JacobianBody(Blist.T, joint_angles)  # Calculate arm Jacobian

            J_robot = np.concatenate((J_base, J_arm), axis=1)  # Concatenate base and arm Jacobians
            J_robot[:, np.where(Joint_Limit_test == False)] = 0  # Apply joint limit test

            return J_robot  # Return full Jacobian
        except Exception as e:
            print(f"Error in calculating Jacobian: {e}", file=sys.stderr)
            return None

    @staticmethod
    def pinv_tol(matrix, tol=1e-7):
        try:
            """
            Calculate the pseudoinverse of a matrix with a tolerance
            """
            matrix[np.abs(matrix) < tol] = 0  # Zero out small values
            return np.linalg.pinv(matrix)  # Return pseudoinverse
        except Exception as e:
            print(f"Error in calculating pseudoinverse: {e}", file=sys.stderr)
            return None
    
    @staticmethod
    def Joint_Limit_test(configuration: RobotConfiguration, ignore=False):
        try:
            """
            Function to test joint limits and avoid singularities
            Inputs : 
            - configuration as instance of RobotConfiguration class
            - ignore (True/False) 
            Outputs : 
            - True or False 
            """
            # Ensure the configuration is an instance of RobotConfiguration
            if not isinstance(configuration, RobotConfiguration):
                raise TypeError("configuration must be an instance of RobotConfiguration")
        
            # Get wheel and joint angles
            wheel_angles = np.array(configuration.get_wheel_angles())  # Convert to numpy array
            joint_angles = np.array(configuration.get_joint_angles())  # Convert to numpy array

            # Print for debugging
            # print("Wheel Angles:", wheel_angles)
            # print("Joint Angles:", joint_angles)

            # Combine wheel and joint angles
            combined = np.concatenate((wheel_angles, joint_angles), axis=None)

            # Initialize test_joint_limits
            test_joint_limits = np.ones(combined.shape, dtype=bool)

            # Find the index of the last wheel angle
            change_arm = wheel_angles.shape[0] - 1

            # Apply joint angle limits check if ignore is False
            if not ignore:
                if joint_angles[2] < -0.03:
                    test_joint_limits[change_arm + 2] = False
                if joint_angles[3] < -0.03:
                    test_joint_limits[change_arm + 3] = False

            return test_joint_limits  # Return joint limits test result
        except Exception as e:
            print(f"Error in joint limit test: {e}", file=sys.stderr)
            return None
    
    @staticmethod
    def Tse_calc(chassi_config, joint_angles):
        try:
            """
            Function to perform calculations for Tse configuration of end effector wrt to {s}
            Inputs : 
            - Chassi_configuration (phi,x,y)
            - Joint angles (j1,j2,j3,j4,j5) of the robot
            Outputs : 
            - Configuration of end effector wrt to {s} Tse
            """
            Tb0 = RobotProperties.Tb0()  # Get base configuration
            M_Home = RobotProperties.M_Home()  # Get home configuration
            Blist = RobotProperties.Blist()  # Get screw axes
            Tsb_calc = RobotProperties.Tsb(chassi_config)  # Calculate chassis configuration
            T0e = mr.FKinBody(M_Home, Blist.T, joint_angles)  # Calculate forward kinematics
            Tse = Tsb_calc @ Tb0 @ T0e  # Calculate end effector configuration
            return Tse  # Return end effector configuration
        except Exception as e:
            print(f"Error in calculating Tse: {e}", file=sys.stderr)
            return None
           
    @staticmethod
    def FeedbackControl(configuration:RobotConfiguration, Tse_d, Tse_d_next, Kp, Ki, Ki_err, dt, test_joint_limits):
        try:
            """
            Main Feedback Controller function
            Inputs :
            - configuration as instance of RobotConfiguration class
            - Tse desired
            - Tse desired next 
            - Kp 
            - Ki
            - Dt 
            - Test joint limits -> Boolean 
            Outputs :
            - Velocities (4 wheel and 5 joint speeds)
            - Robot Jacobian J_robot
            - Ki_error integral error 
            - Twist 
            """
            chassi_config = configuration.get_chassi_configuration()  # Get chassis configuration
            joint_angles = configuration.get_joint_angles()  # Get joint angles
            Tse = Controller.Tse_calc(chassi_config, joint_angles)  # Calculate current end effector configuration

            if Tse is None:
                raise ValueError("Invalid Tse configuration.")

            # Calculate error between current and desired configuration
            Xerr = mr.se3ToVec(mr.MatrixLog6(np.dot(Controller.pinv_tol(Tse), Tse_d)))
            # Calculate adjoint transformation
            Ad = mr.Adjoint(np.dot(Controller.pinv_tol(Tse), Tse_d))
            # Calculate desired twist
            Twist = mr.se3ToVec((1/dt) * mr.MatrixLog6(np.dot(Controller.pinv_tol(Tse_d), Tse_d_next)))

            # Update integral error
            Ki_err = Ki_err + Ki * Xerr * dt
            # Calculate control law
            V_t = np.dot(Ad, Twist) + np.dot(Kp, Xerr) + Ki_err

            # Calculate robot Jacobian
            J_robot = Controller.Jacobian_robot(configuration, test_joint_limits)
            if J_robot is None:
                raise ValueError("Invalid robot Jacobian.")

            # Calculate velocities
            velocities = np.dot(Controller.pinv_tol(J_robot), V_t)

            return velocities, Xerr, Ki_err, J_robot, V_t, Twist  # Return calculated values
        except Exception as e:
            print(f"Error in feedback control: {e}", file=sys.stderr)
            return None, None, None, None, None, None
