"""
04-05-2024
Capstone Project - Modern robotics
FeedbackController.py : Uses reference trajectory and calculates the joint
and wheel velocities to get robot to desired configuration

@ Alexandros Mantzios
"""

import sys
import numpy as np
import core as mr
from math import cos, sin
from Configurations import RobotConfiguration, RobotProperties


class Controller:
    """
    Feedback Controller for mobile manipulator.
    Receives a RobotProperties instance at construction to cache geometry.
    """

    def __init__(self, robot_props: RobotProperties):
        # Cache robot geometry so we don't re-fetch every call
        self.Tb0 = robot_props.Tb0
        self.M_Home = robot_props.M_Home
        self.Blist = robot_props.Blist
        self.joint_limits = robot_props.joint_limits
        self.r = robot_props.r
        self.l = robot_props.l
        self.omega = robot_props.omega
        self.height = robot_props.height
        self.F6 = robot_props.F6
        self.robot_props = robot_props  # Keep reference for Tsb()

    def Jacobian_robot(self, configuration, Joint_Limit_test):
        """
        Jacobian for mobile robot base
        Inputs :
        - configuration : RobotConfiguration instance (13 element vector)
        - Joint_Limit_test result (boolean array)
        Outputs :
        - J_robot -> 6x9 Jacobian for entire robot (4 wheel + 5 joint)
        """
        try:
            joint_angles = configuration.get_joint_angles()

            # Robot arm base with respect to end effector
            T0e = mr.FKinBody(self.M_Home, self.Blist.T, joint_angles)  # Calculate forward kinematics

            # Calculate adjoint transformation between base and end effector
            Teb = np.dot(mr.TransInv(T0e), mr.TransInv(self.Tb0))
            J_base = np.dot(mr.Adjoint(Teb), self.F6)  # Calculate base Jacobian (6x4)

            # Calculate arm body Jacobian (6x5)
            J_arm = mr.JacobianBody(self.Blist.T, joint_angles)

            # Concatenate base and arm Jacobians (6x9)
            J_robot = np.concatenate((J_base, J_arm), axis=1)
            # Apply joint limit test - zero out constrained columns
            J_robot[:, np.where(Joint_Limit_test == False)] = 0

            return J_robot
        except Exception as e:
            print(f"Error in calculating Jacobian: {e}", file=sys.stderr)
            return None

    @staticmethod
    def pinv_tol(matrix, tol=1e-7):
        """
        Calculate the pseudoinverse of a matrix with a tolerance.
        Zeroes out values smaller than tol before computing pseudoinverse.
        Works on a copy to avoid mutating the input.
        """
        try:
            matrix_copy = matrix.copy()
            matrix_copy[np.abs(matrix_copy) < tol] = 0
            return np.linalg.pinv(matrix_copy)
        except Exception as e:
            print(f"Error in calculating pseudoinverse: {e}", file=sys.stderr)
            return None

    def Joint_Limit_test(self, configuration: RobotConfiguration, ignore=False):
        """
        Function to test joint limits and avoid singularities
        Inputs :
        - configuration as instance of RobotConfiguration class
        - ignore (True/False)
        Outputs :
        - 9-element boolean array (4 wheels + 5 joints)
        """
        try:
            if not isinstance(configuration, RobotConfiguration):
                raise TypeError("configuration must be an instance of RobotConfiguration")

            # Get wheel and joint angles
            wheel_angles = np.array(configuration.get_wheel_angles())
            joint_angles = np.array(configuration.get_joint_angles())

            # Combine wheel and joint angles
            combined = np.concatenate((wheel_angles, joint_angles), axis=None)

            # Initialize test_joint_limits - all True
            test_joint_limits = np.ones(combined.shape, dtype=bool)

            # Find the index offset where arm joints start
            change_arm = wheel_angles.shape[0] - 1

            # Apply joint angle limits check for all 5 joints
            if not ignore:
                for i, (lower, upper) in enumerate(self.joint_limits):
                    if joint_angles[i] < lower or joint_angles[i] > upper:
                        test_joint_limits[change_arm + 1 + i] = False

            return test_joint_limits
        except Exception as e:
            print(f"Error in joint limit test: {e}", file=sys.stderr)
            return None

    def Tse_calc(self, chassi_config, joint_angles):
        """
        Function to perform calculations for Tse configuration of end effector wrt to {s}
        Tse = Tsb @ Tb0 @ T0e
        Inputs :
        - Chassi_configuration (phi, x, y)
        - Joint angles (j1, j2, j3, j4, j5) of the robot
        Outputs :
        - Configuration of end effector wrt to {s} Tse
        """
        try:
            Tsb_calc = self.robot_props.Tsb(chassi_config)  # Calculate chassis configuration
            T0e = mr.FKinBody(self.M_Home, self.Blist.T, joint_angles)  # Calculate forward kinematics
            Tse = Tsb_calc @ self.Tb0 @ T0e  # Calculate end effector configuration
            return Tse
        except Exception as e:
            print(f"Error in calculating Tse: {e}", file=sys.stderr)
            return None

    def FeedbackControl(self, configuration: RobotConfiguration, Tse_d, Tse_d_next,
                        Kp, Ki, Ki_err, dt, test_joint_limits):
        """
        Main Feedback Controller function (PI control law)
        Inputs :
        - configuration as instance of RobotConfiguration class
        - Tse_d : desired end effector configuration
        - Tse_d_next : next desired end effector configuration
        - Kp : proportional gain (6x6)
        - Ki : integral gain (scalar)
        - Ki_err : accumulated integral error
        - dt : time step
        - test_joint_limits : boolean array from Joint_Limit_test
        Outputs :
        - Velocities (4 wheel and 5 joint speeds)
        - Xerr : 6-element twist error
        - Ki_err : updated integral error
        - J_robot : robot Jacobian
        - V_t : commanded twist
        - Twist : feedforward twist
        """
        try:
            chassi_config = configuration.get_chassi_configuration()
            joint_angles = configuration.get_joint_angles()
            Tse = self.Tse_calc(chassi_config, joint_angles)  # Current end effector config

            if Tse is None:
                raise ValueError("Invalid Tse configuration.")

            # Calculate error between current and desired configuration
            Xerr = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tse), Tse_d)))
            # Calculate adjoint transformation for frame change
            Ad = mr.Adjoint(np.dot(mr.TransInv(Tse), Tse_d))
            # Calculate desired feedforward twist
            Twist = mr.se3ToVec((1 / dt) * mr.MatrixLog6(np.dot(mr.TransInv(Tse_d), Tse_d_next)))

            # Update integral error
            Ki_err = Ki_err + Ki * Xerr * dt
            # Calculate control law: V = Ad*Vd + Kp*Xerr + Ki_err
            V_t = np.dot(Ad, Twist) + np.dot(Kp, Xerr) + Ki_err

            # Calculate robot Jacobian
            J_robot = self.Jacobian_robot(configuration, test_joint_limits)
            if J_robot is None:
                raise ValueError("Invalid robot Jacobian.")

            # Calculate velocities via pseudoinverse
            velocities = np.dot(Controller.pinv_tol(J_robot), V_t)

            return velocities, Xerr, Ki_err, J_robot, V_t, Twist
        except Exception as e:
            print(f"Error in feedback control: {e}", file=sys.stderr)
            return None, None, None, None, None, None
