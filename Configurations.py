"""
20-05-2024
Capstone Project - Modern Robotics
Configuration.py : Includes classes for robot configuration and robot geometry
Includes
- Set configuration as np.array
- Set gripper status
- Retrieve robot joint angles and speeds
- Retrieve mobile robot chassi configuration and speeds
- Configuration of mobile robot base relative to arm base Tbo
- Configuration for mobile robot base relative to space (world) Tbs
- Robot Home configuration
- Robot screw matrix Blist
- Configuration of cube frame relative to space (world) - Initial
- Configuration of cube frame relative to space (world) - Final

@ Alexandros Mantzios
"""

import numpy as np
import core as mr


class RobotConfiguration:
    """
    Robot configuration as a 13-vector:
    (chassi φ, chassi x, chassi y, j1_theta, j2_theta, j3_theta, j4_theta,
    j5_theta, w1_theta, w2_theta, w3_theta, w4_theta, gripper)
    """

    def __init__(self, array: np.ndarray):
        if len(array) != 13:
            raise ValueError(f"Expected 13-element array, got {len(array)}")
        self.configuration = array

    def get_configuration_as_array_(self):
        """Returns configuration as array"""
        return self.configuration

    def set_gripper_status(self, gripper):
        """Sets the gripper status for open and closed (1-closed/0-open)"""
        self.configuration[12] = gripper

    def get_joint_angles(self):
        """Retrieves robot joint angles (4-7 elements of array)"""
        return self.configuration[3:8]

    def get_wheel_angles(self):
        """Retrieves wheel angles"""
        return self.configuration[8:12]

    def get_chassi_configuration(self):
        """Retrieves chassi configuration (1-3 elements of array)"""
        return self.configuration[:3]

    def get_all_angles(self):
        """Returns all joint and wheel angles"""
        return self.configuration[3:12]


class RobotProperties:
    """
    Contains Robot geometry details such as
        - Configuration matrix T b->0
        - Configuration matrix T s->b
        - Robotic arm home configuration M
        - Screw matrix list Blist
        - Joint limits (estimated)
        - Pre-computed mobile base kinematics matrix F

    # Frames
    #{s} fixed space
    #{b} mobile base body frame
    #{0} manipulator base frame
    #{e} end effector frame
    #{c} cube frame
    #{g} goal frame
    """

    def __init__(self):
        # wheel radius in meters
        self.r = 0.0475
        # distance to wheel axis
        self.l = 0.235
        # half wheel axis
        self.omega = 0.15
        # height of base from floor
        self.height = 0.0963

        # Configuration of frame {b} chassi relative to {0} KUKA base
        self.Tb0 = np.array([[1, 0, 0, 0.1662],
                             [0, 1, 0, 0],
                             [0, 0, 1, 0.0026],
                             [0, 0, 0, 1]])

        # Initial Configuration of frame {e} end effector relative to {s} space
        self.Tse_init = np.array([[0, 0, 1, 0],
                                  [0, 1, 0, 0],
                                  [-1, 0, 0, 0.5],
                                  [0, 0, 0, 1]])

        # Home configuration of robot
        self.M_Home = np.array([[1, 0, 0, 0.033],
                                [0, 1, 0, 0],
                                [0, 0, 1, 0.6546],
                                [0, 0, 0, 1]])

        # List of screw axes of robot
        self.Blist = np.array([[0, 0, 1, 0, 0.033, 0],
                               [0, -1, 0, -0.5076, 0, 0],
                               [0, -1, 0, -0.3526, 0, 0],
                               [0, -1, 0, -0.2176, 0, 0],
                               [0, 0, 1, 0, 0, 0]])

        # Recommended joint limits to avoid singularities
        self.joint_limits = [[-0.9, 0.9],
                             [-1.8, 1.1],
                             [-0.2, 0.2],
                             [-0.2, 0.2],
                             [-2.85, 2.85]]

        # Pre-compute mobile base kinematics matrix F
        r, l, w = self.r, self.l, self.omega
        H0 = (1 / r) * np.array([[-l - w, 1, -1],
                                  [l + w, 1, 1],
                                  [l + w, 1, -1],
                                  [-l - w, 1, 1]])
        self.F = np.linalg.pinv(H0)  # 3x4 pseudoinverse of H0

        # F embedded in 6D twist space (6x4)
        self.F6 = np.zeros((6, 4))
        self.F6[2:5, :] = self.F

    def Tsb(self, q):
        """
        Frame {b} chassi relative to {s} space
        Input:
        q - configuration of robot (phi, x, y)
        """
        phi, x, y = q[0], q[1], q[2]
        return np.array([[np.cos(phi), -np.sin(phi), 0, x],
                         [np.sin(phi),  np.cos(phi), 0, y],
                         [0, 0, 1, self.height],
                         [0, 0, 0, 1]])


class Cube_Task:
    """
    Contains cube frame information
    - Initial cube configuration in space
    - Goal cube configuration in space
    - EE grip and standoff poses relative to cube
    """

    @staticmethod
    def Tsc_init():
        """Initial configuration of frame {c} of cube relative to {s} space"""
        return np.array([[1, 0, 0, 1],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0.025],
                         [0, 0, 0, 1]])

    @staticmethod
    def Tsc_init_new():
        """Initial configuration of frame {c} - new task"""
        return np.array([[1, 0, 0, 1.5],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0.025],
                         [0, 0, 0, 1]])

    @staticmethod
    def Tsc_goal():
        """
        Goal configuration of frame {c} cube relative to {s} space
        Used for best and overshoot
        """
        return np.array([[0, 1, 0, 0],
                         [-1, 0, 0, -1],
                         [0, 0, 1, 0.025],
                         [0, 0, 0, 1]])

    @staticmethod
    def Tsc_goal_new():
        """Goal configuration of frame {c} cube relative to {s} space - new task"""
        return np.array([[0, 1, 0, 0],
                         [-1, 0, 0, -1.5],
                         [0, 0, 1, 0.025],
                         [0, 0, 0, 1]])

    @staticmethod
    def Tce_grip():
        """Configuration of frame {e} EE relative to {c} cube - grip pose"""
        s = 1 / np.sqrt(2)
        return np.array([[-s, 0, s, 0],
                         [0, 1, 0, 0],
                         [-s, 0, -s, 0],
                         [0, 0, 0, 1]])

    @staticmethod
    def Tce_standoff():
        """Configuration of frame {e} EE relative to {c} cube - standoff pose"""
        s = 1 / np.sqrt(2)
        return np.array([[-s, 0, s, 0],
                         [0, 1, 0, 0],
                         [-s, 0, -s, 0.1],
                         [0, 0, 0, 1]])


class Parameters:
    """
    Contains initial parameters for linear velocity,
    acceleration, solver coefficients, trajectory types etc
    """

    # maximum linear velocity 200mm/s
    max_linear_velocity = 0.2
    # maximum angular velocity 20 rad/s
    max_angular_velocity = 20
    # time scales - Cubic = 3, Quintic = 5
    time_scale = 3
    # Trajectory types - "CartesianTrajectory" or "ScrewTrajectory"
    trajectory_type = "CartesianTrajectory"

    @staticmethod
    def k():
        return 1
