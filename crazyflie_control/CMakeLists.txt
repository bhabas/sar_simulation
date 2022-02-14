cmake_minimum_required(VERSION 3.0.2)
project(crazyflie_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  crazyflie_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES 
 CATKIN_DEPENDS roscpp 
#  DEPENDS 
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pub_example1 src/pub_example1.cpp)
add_dependencies(pub_example1 ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub_example1 ${catkin_LIBRARIES})