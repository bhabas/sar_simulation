cmake_minimum_required(VERSION 3.0.2)
project(crazyflie_gazebo2)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crazyflie_gazebo2
#  CATKIN_DEPENDS gazebo_ros
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

