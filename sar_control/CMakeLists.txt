cmake_minimum_required(VERSION 3.0.2)
project(sar_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  crazyflie_msgs
)

catkin_package(
 INCLUDE_DIRS include include_CF_Firmware
#  LIBRARIES
 CATKIN_DEPENDS roscpp 
#  DEPENDS 
)

include_directories(include include_CF_Firmware ${catkin_INCLUDE_DIRS})

add_executable(controller src/stabilizer.cpp src/controller_GTC.c)
add_dependencies(controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES})
target_compile_definitions(controller PRIVATE CONFIG_SAR_SIM CONFIG_CONTROLLER_OOT)


# add_executable(controller src/stabilizer.cpp src/controller_gtc.c src/estimator.cpp src/shared_lib.c src/nml.c src/nml_util.c)
# add_dependencies(controller ${catkin_EXPORTED_TARGETS})
# target_link_libraries(controller ${catkin_LIBRARIES})
# target_compile_definitions(controller PRIVATE GAZEBO_SIM)

