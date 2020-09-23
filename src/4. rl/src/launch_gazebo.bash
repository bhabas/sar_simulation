#!/bin/bash
killall gzserver gzclient
cd ~/catkin_ws
source devel/setup.bash
source src/crazyflie_simulation/src/crazyflie_gazebo/setup_gazebo.bash $(pwd) $(pwd)/devel/lib
# source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
roslaunch crazyflie_gazebo crazyflie_sitl.launch world_name:="ceiling" vehicle:="crazyflie_landing_gears" gui:="True"
