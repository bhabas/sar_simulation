# Small Aerial Robot Perching Project


Overview
------------------------
The package provided is the library for our Small Aerial Robot Perching Project, which focuses on landing small quadrotor robots on surfaces of various orientations using Deep Reinforcement Learning and other machine learning methods. The library is designed to run within the Gazebo physics engine and ROS. Please note that this library is under active development.


## Contributors

- **Developer/Maintainer:** Bryan Habas, (BHabas@psu.edu)
  - **Affiliation:** [BioRob-InFl](https://sites.psu.edu/infl/), PSU



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


## Environment

* Operating System
  * [Ubuntu 20.04](http://releases.ubuntu.com/focal/) 
* Middleware
  * [ROS1 Noetic](http://wiki.ros.org/Installation/Ubuntu) 
* Physics Simulation
  * [Gazebo Citadel](https://gazebosim.org/docs/latest/ros_installation) 


## Download and Setup
### 1 - Install ROS
- Install ROS Noetic: http://wiki.ros.org/Installation/Ubuntu
### 2 - Create a catkin workspace
- Create catkin_ws dir: https://wiki.ros.org/catkin/Tutorials/create_a_workspace
```
cd ~/catkin_ws/src
git clone git@github.com:bhabas/crazyflie_simulation.git
```
### 3 - Clone the git repository
- Install repo:
```
cd ~/catkin_ws/src
git clone git@github.com:bhabas/crazyflie_simulation.git
cd crazyflie_simulation
```
### 4 - Install the dependencies missing (ardrone_autonomy) package
```
pip install -e .
```

## Compile

### 1 - Compile the ros package with catkin
- Compile package:
```
sh  SAR_Perching_Install.sh
```




## Run Simulation Playground:
This is a general playground area for testing purposes. It will launch the Gazebo simulation environment with a quadrotor drone and landing surface. Various controls and actions can be activated via numeric command in the terminal window

Initialize roscore in a seperate terminal:
```
roscore
```

Launch Simulation Playground:
```
rosrun sar_general Control_Playground.py
```

## Deep Reinforcement Learning Example
Initialize roscore in a seperate terminal:
```
roscore
```

Launch Deep RL Script:
```
rosrun sar_general Control_Playground.py
```



## Config File

Sim_Settings.yaml

```
SAR_SETTINGS:
  SAR_Type - Quadrotor Classification
  SAR_Config -  Quadrotor Model/Configuration
  Policy_Type - Policy Types [PARAM_OPTIM,SVL_POLICY,DEEP_RL_SB3,DEEP_RL_ONBD]

PLANE_SETTINGS:
  Plane_Model - Plane Model
  Plane_Config - Plane Orientation [deg]

SIM_SETTINGS:
  GUI_Flag - Flag to enable/disable Gazebo GUI
  Pause_Flag - Flag to start simulation paused/unpaused
  Sim_Speed - Simulation run speed
  Sim_Slowdown_Speed - Simulaiton speed near contact
  Landing_Slowdown_Flag - Disable Sim_Slowdown_Speed

SAR_DC_SETTINGS:
  Logging_Rate - Recording rate of logging data [Hz]

```





## Troubleshooting:



Useful Links:
VSCode C++ setup: https://dev.to/talhabalaj/setup-visual-studio-code-for-multi-file-c-projects-1jpi



## Run Steps

```
roscore # in different terminal tab
./catkin_ws/src/crazyflie_simulation catkin_make
tensorboard --logdir /home/sashayaskolko/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Leg_Design_Analysis/TB_Logs/CF_Gazebo
/bin/python3 /home/sashayaskolko/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Leg_Design_Analysis/Policy_Training_DeepRL.py
```