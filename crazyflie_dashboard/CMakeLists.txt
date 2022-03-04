cmake_minimum_required(VERSION 3.0.2)
project(crazyflie_dashboard)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crazyflie_dashboard
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
