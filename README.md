# crazyflie_simulation

Troubleshooting:

[Err] [REST.cc:205] Error in REST request
https://answers.gazebosim.org//question/25030/gazebo-error-restcc205-error-in-rest-request/


clone respository into ~/catkin_ws/src. Change username in rl file and env.py file to local username. Everything should work as is

Useful Links:
VSCode C++ setup: https://dev.to/talhabalaj/setup-visual-studio-code-for-multi-file-c-projects-1jpi



Useful commands:

roscore
catkin_ws/src/crazyflie_simulation$ catkin_make
tensorboard --logdir /home/sashayaskolko/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Leg_Design_Analysis/TB_Logs/CF_Gazebo
/bin/python3 /home/sashayaskolko/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Leg_Design_Analysis/Policy_Training_DeepRL.py