cmake_minimum_required(VERSION 3.0.2)
project(crazyflie_data_converter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  crazyflie_msgs_exp
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES cf_data_converter
 CATKIN_DEPENDS roscpp rospy 
#  DEPENDS system_lib
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(CF_DataConverter src/CF_DataConverter.cpp)
add_dependencies(CF_DataConverter ${catkin_EXPORTED_TARGETS})
target_link_libraries(CF_DataConverter ${catkin_LIBRARIES})

