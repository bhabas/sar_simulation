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

add_executable(SAR_Controller src/stabilizer.cpp src/controller_GTC.c src/shared_lib.c src/CompressStates.c src/traj_funcs.c)
add_dependencies(SAR_Controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(SAR_Controller ${catkin_LIBRARIES})
target_compile_definitions(SAR_Controller PRIVATE CONFIG_SAR_SIM CONFIG_CONTROLLER_OOT)

