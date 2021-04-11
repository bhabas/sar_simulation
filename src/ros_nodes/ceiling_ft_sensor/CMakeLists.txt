cmake_minimum_required(VERSION 3.0.2)
project(ceiling_ft_sensor)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
find_package(catkin REQUIRED geometry_msgs roscpp std_msgs)
find_package(gazebo REQUIRED)


                     
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package(
   #INCLUDE_DIRS include
   LIBRARIES ft_sensor ${GAZEBO_LIBRARIES}
   CATKIN_DEPENDS geometry_msgs roscpp std_msgs
#  DEPENDS system_lib
)


link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(ceiling_ft_sensor src/ceiling_ft_rostopic_converter.cpp)

target_link_libraries(ceiling_ft_sensor ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
 
