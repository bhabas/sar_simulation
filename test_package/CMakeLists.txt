cmake_minimum_required(VERSION 3.0.2)
project(test_package)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  crazyflie_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES cf_data_converter
 CATKIN_DEPENDS roscpp rospy 
#  DEPENDS system_lib
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(CFDC src/CF.cpp src/bar.cpp)
add_dependencies(CFDC ${catkin_EXPORTED_TARGETS})
target_link_libraries(CFDC ${catkin_LIBRARIES} ${CURSES_LIBRARIES})

