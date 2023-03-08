# Small Aerial Robot Perching Project


Overview
------------------------
The package provided is the library for our Small Aerial Robot Perching Project, which focuses on landing small quadrotor robots on surfaces of various orientations using Deep Reinforcement Learning and other machine learning methods. The library is designed to run within the Gazebo physics engine and ROS. Please note that this library is under active development.


## Contributors

- **Developer/Maintainer:** Bryan Habas, (BHabas@psu.edu)
  - **Affiliation:** [BioRob-InFl](https://sites.psu.edu/infl/)



## License
```
Copyright 2023 BioRob-InFl Lab

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE..
    
```
## Citation
In the event that you include this work in your publication, we kindly request that you cite our paper as a reference: 
```
@inproceedings{habas2022optimal,
  title={Optimal Inverted Landing in a Small Aerial Robot with Varied Approach Velocities and Landing Gear Designs},
  author={Habas, Bryan and AlAttar, Bader and Davis, Brian and Langelaan, Jack W and Cheng, Bo},
  booktitle={2022 International Conference on Robotics and Automation (ICRA)},
  pages={2003--2009},
  year={2022},
  organization={IEEE}
}

@article{habas2022inverted,
  title={Inverted Landing in a Small Aerial Robot via Deep Reinforcement Learning for Triggering and Control of Rotational Maneuvers},
  author={Habas, Bryan and Langelaan, Jack W and Cheng, Bo},
  journal={arXiv preprint arXiv:2209.11043},
  year={2022}
} 
 ```
 


 ## Acknowledgments

This research was made possible by the National Science Foundation grants IIS-1815519 and CMMI-1554429 awarded to B.C. and his [BioRob-InFl](https://sites.psu.edu/infl/) Lab, as well as the support of the Department of Defense (DoD) through the National Defense Science \& Engineering Graduate (NDSEG) Fellowship Program awarded to B.H.. We would like to express our appreciation to these organizations for their support of our research efforts.


Installation  
-------------------------
```
git clone https://github.com/arplaboratory/TrajPlanARPL
git clone https://github.com/arplaboratory/arpl_msgs.git
cd ARPL_Trajectory_Planning
sh install_trajgen_depend.sh
```


Usage
------------------------

ON YOUR LAPTOP, clone and build the repository Waypoint Navigation Plug-in (usefull for GUI):

```
git clone https://github.com/KumarRobotics/waypoint_navigation_plugin.git
```

Run Demo External 
------------------------
please follow these steps on your computer:
```
roslaunch waypoint_navigation_plugin rviz.launch
```
Open another terminal and run

```
roslaunch ros_traj_gen_utils traj_plan.launch
```

Change in waypoint navigation plugin topic: to vehicle_name/waypoints 
vehicle_name is in the .yaml 
Load the waypoints in the ros_traj_gen_utils/config/perch_general.bag
The plugin will publish a nav_msgs/path
Output visualized 3D path. 
topic published on vehicle_name/position_cmd

This is for general flight. If you want to use perching simpling write.


```
roslaunch ros_traj_gen_utils perch.launch
```

Initially this is set with perch_config target orientation of 90 degrees where you should see it reflected in the acceleration.
You can also drag and drop waypoints to see various paths.


Launch File
------------------------
The launch file loads std_launch.yaml running  roslaunch ros_traj_gen_utils traj_plan.launch

Config File
------------------------
config.yaml
Standard config
mav_name - vehicle if you want to loop a vehicle's odometry mav_name/odom inside nav_msgs/odometry type. If no odometry is detected it will not automatically lopp the mav_name/odom topic inside
replan - boolean true/false allows replanning

perch_config.yaml
visual - boolean true/false. Allows visual feedback to pushed in.
Will expect Geometry_msgs/Array for AprilTagDetection on topic name "/tag_detections_pose" to use that pose.

target - 4x4 matrix example given. Only the first 3x3 block. Will set a desired target if you want to hardcore it. This will be applied to the last target.If no target is set then we assume a full stop. Will be overrieded by visual if visual is enabled to true. 

Currently, this is naturally set to a 90 degree pitch interception. 


Library Features
------------------------
Traj_gen includes
Folder include contains the .h files it is divided into 3 folders. 
  *  ooqp_interface - contains the .h files for interacting with OOQP - OoqpEigenInterface.hpp is the main one ot look at
  *  trajectory - contains the various trajectory formulations

Trajectories 
  *  QPpolyTraj.h -> Contains 2 solvers for a QP formulation of the function of a polynomial based trajectories
        *  Fast Solve -> Solves 2x ->10x faster than Normal Solve but creates a slightly less optimal path
        *  Solve -> Basic Solve that finds the optimal path that minimizes snap for a polynomial

ros_traj_gen_utils
Contains ROS interfaces.


COMMON INSTALLATION PROBLEMS
------------------------
99.99% of install problems especially if you get errors like involve gfortran directory being wrong
ma27d.f:(.text+0x59cc): undefined reference to `_gfortran_st_write'
Go to the /traj_gen/cmake/Findooqp.cmake file and edit line 105 down and set the GFORTRAN_DIR to the correct place you installed gfortran.so
You can also use dpkg -L gfortran command to find it 



Troubleshooting:

[Err] [REST.cc:205] Error in REST request
https://answers.gazebosim.org//question/25030/gazebo-error-restcc205-error-in-rest-request/


clone respository into ~/catkin_ws/src. Change username in rl file and env.py file to local username. Everything should work as is

Useful Links:
VSCode C++ setup: https://dev.to/talhabalaj/setup-visual-studio-code-for-multi-file-c-projects-1jpi

# Build Steps

- Install roscore: http://wiki.ros.org/Installation/Ubuntu
- Make catkin_ws: https://wiki.ros.org/catkin/Tutorials/create_a_workspace
- Run `catkin_make` in `./catkin_ws`
- Go into `./catkin_ws/src/` and `git clone git@github.com:bhabas/crazyflie_simulation.git`
- Temporarily move `crazyflie_gazebo` from git repository
- Run `catkin_make` in `./catkin_ws`
- Readd `crazyflie_gazebo` into `crazyflie_simulation` folder
- Run `catkin_make` in `./catkin_ws`


# Run Steps

```
roscore # in different terminal tab
./catkin_ws/src/crazyflie_simulation catkin_make
tensorboard --logdir /home/sashayaskolko/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Leg_Design_Analysis/TB_Logs/CF_Gazebo
/bin/python3 /home/sashayaskolko/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Leg_Design_Analysis/Policy_Training_DeepRL.py
```