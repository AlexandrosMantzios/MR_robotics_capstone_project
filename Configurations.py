"""
20-05-2024
Capstone Project - Moder robotitcs 
Configuration.py : Includes classes for robot configuration and robot geometry
Includes 
- Set configuration as np.array
- Set gripper status
- Retrieve robot joint angles and speeds
- Retrieve mobile robot chassi configuration and speeds
- Configuration of mobile robot base  relative to arm base Tbo
- Configuration for mobile robot base relative to space (world) Tbs
- Robot Home configuration 
- Robot screw matrix Blist
- Configuration of cube frame relative to space (world) - Initial
- Configuration of cube frame relative to space (world) - Final

@ Alexandros Mantzios

"""

import numpy as np 
import modern_robotics



class RobotConfiguration: 

    """ 
    Robot configuration must be a 13-vector comprised of the following
     (chassi φ, chassi x, chassi, y, j1_theta, j2_theta, j3_theta, j4_theta
    j5_theta, w1_theta, w2_theta, w3_theta, w3_theta, 

    """

    def __init__ (self, array:np.ndarray):
        self.configuration=array

    # def __init__(self, configuration):
    #     self.configuration = configuration

    def get_configuration_as_array_(self):
        """
        Returns configuration as array
        """
        return self.configuration
    
    def set_gripper_status (self, gripper):
        """
        Sets the gripper status for open and closed (1-closed/0-open)
        """
        self.configuration[12]= gripper

    def get_joint_angles(self):
        """
        Retrieves robot joint angles (4-7 elements of array)
        """
        return self.configuration[3:8]
    
    def get_wheel_angles(self):
        """
        Retrieves wheel angles
        """
        return self.configuration[8:12]

    def get_chassi_configuration(self):
        """
        Retrieves chassi configuration (1-3 elements of array)
        """
        #assert len(self.configuration) == 13
        return self.configuration[:3]
    
    def get_all_angles(self):
        """
        Returns all joint and wheel angles
        """
        #assert len(self.configuration) == 13
        return self.configuration[3:12]
    

class RobotProperties: 
    """ 
    Contains Robot geometry details such as 
        - Configuration matrix T b->0
        - Configuration matrix T s->b
        - Robotic arm home configuration M
        - Srcew matrix list Blist
        - Joint limits (estimated)
    
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
        self.ω = 0.15
        # height of base from floor
        self.height = 0.0963

    @staticmethod
    def Tb0():
        """
        Configuration of frame {b} chassi relative to {0} KUKA base
        """
        return np.array([[1,0,0,0.1662],
                         [0,1,0,0],
                         [0,0,1,0.0026],
                         [0,0,0,1]])
    @staticmethod
    def Tse_init():
        """
        Initial Configuration of frame {e} end effector relative to {s} space
        """
        return np.array([[0,0,1,0],
                      [0,1,0,0],
                      [-1,0,0,0.5],
                      [0,0,0,1]])
    
    @staticmethod
    def Tsb(q): 
        """
        Frame {b} chassi relative to {s} space
        Input:
        q configuration of robot
        """
        φ = q[0]
        x = q[1]
        y = q[2]
        z = 0.0963
        Ts_b = np.array([[np.cos(φ),-np.sin(φ),0,x],
                         [np.sin(φ),np.cos(φ),0,y],
                         [0,0,1,z],
                         [0,0,0,1]])
        return Ts_b

    @staticmethod
    def M_Home():
        """
        Home configuration of robot
        """ 
        return np.array([[1,0,0,0.033],
                         [0,1,0,0],
                         [0,0,1,0.6546],
                         [0,0,0,1]])
    



    @staticmethod 
    def Blist():
        """
        List of screw axes of robot
        """
        #List of screw axes of the robot
        return np.array([[0,0,1,0,0.033,0],
                        [0,-1,0,-0.5076,0,0],
                        [0,-1,0,-0.3526,0,0],
                        [0,-1,0,-0.2176,0,0],
                        [0,0,1,0,0,0],])
 

    




    @staticmethod
    def joint_limits(): 
         """
         Recommended joint limits to avoid singularities
         """
         return [[-0.9,0.9],
                 [-1.8, 1.1],
                 [-0.2, 0.2],
                 [-0.2, 0.2],
                 [-2.85, 2.85]]



class Cube_Task:
    """
    Contains cube frame information
    - Initial cube configuration in space 
    - Goal cube configuration in space 

    """

    @staticmethod
    def Tsc_init():
        """
        Initial configuration of frame {c} of cube relative to {s} space
        """
        return np.array([[1,0,0,1],
                         [0,1,0,0],
                         [0,0,1,0.025],
                         [0,0,0,1]])
    
    # # New Task Settings
    def Tsc_init_new():
        
        return np.array([[1,0,0,1.5],
                  [0,1,0,0],
                  [0,0,1,0.025],
                  [0,0,0,1]])
    
 
    @staticmethod
    def Tsc_goal():
        """
        Goal configuration of frame {c} cube relative to {s} space
        Used for best and overshoot 
        """
        return np.array([[0,1,0,0],
                         [-1,0,0,-1],
                         [0,0,1,0.025],
                         [0,0,0,1]])
    
  
    @staticmethod
    def Tsc_goal_new():
        """
        Goal configuration of frame {c} cube relative to {s} space
        Uncomment for 
        """
        return np.array([[0,1,0,0],
                      [-1,0,0,-1.5],
                      [0,0,1,0.025],
                      [0,0,0,1]])

    @staticmethod
    def Tce_grip():
        """
        Configuration of frame {e} EE relative to {c} cube 
        """
        return np.array([[-1/np.sqrt(2), 0, 1/np.sqrt(2), 0],
						  [0, 1, 0, 0],
						  [-1/np.sqrt(2), 0, -1/np.sqrt(2), 0],
						  [0, 0, 0, 1]])
    @staticmethod
    def Tce_standoff():   
        """
        Configuration of frame {e} EE relative to {c} cube
        """ 
        return np.array([[-1/np.sqrt(2), 0, 1/np.sqrt(2), 0],
						  [0, 1, 0, 0],
						  [-1/np.sqrt(2), 0, -1/np.sqrt(2), .1],
						  [0, 0, 0, 1]])


class Parameters:
    """
    Contains initial parameters for linear velocity
    acceleration, solver coefficients, trajectory types etc
    
    """
    
    # maximum linear velocity 250mm/s
    max_linear_velocity = 0.2
    # maximum angular velocity 30deg/s
    max_angular_velocity = 20
    # time scales
    # Cubic = 3 , Quantic = 5
    # time_scale = 3
    time_scale = 3
    # Trajectory types 
    #trajectory_type = "ScrewTrajectory"
    trajectory_type = "CartesianTrajectory"

    def k():
        return 1  # Example implementation of the k() method returning a value
    
    # Task selection 
    
    
