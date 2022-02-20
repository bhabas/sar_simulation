cmake_minimum_required(VERSION 3.0.2)
project(crazyflie_control2)

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


add_executable(stabilizer src/stabilizer.cpp src/controller_gtc.c src/power_distribution_stock.cpp src/estimator.cpp src/nml.c src/nml_util.c)
add_dependencies(stabilizer ${catkin_EXPORTED_TARGETS})
target_link_libraries(stabilizer ${catkin_LIBRARIES})


