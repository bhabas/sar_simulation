#!/bin/bash
cd ~/src/Firmware
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl2.launch fcu_url:="udp://:14540@127.0.0.1:14557" world_name:="ceiling" vehicle:="crazyflie_landing_gears"
