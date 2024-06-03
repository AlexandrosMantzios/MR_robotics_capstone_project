Capstone Project - Alexandros Mantzios 

main.py : Combines  Next_State.py, FeedbackController.py and Trajectories.py
and outputs a result_AM csv file for the coppelia animation. 

Basic functionality 
- Calculates the control law using FeedbackControl and generates control velocities
- Sends controls, configuration and timestep to NextState and calculates new configuration.
- Stores configurations for later animations 
- Shows error over time


Next_State.py : Module to compute the next robot configuration on a given configuration


FeedbackController.py : Uses reference trajectory and calculates the joint 
and wheel velocities to get robot to desired configuration


Trajectories.py : Module to compute the trajectory segments between T_initial 
and T_goal. Creates trajectory and stores values. 


