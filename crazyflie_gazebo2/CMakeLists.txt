cmake_minimum_required(VERSION 3.0.2)
project(crazyflie_gazebo2)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  crazyflie_msgs
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    gazebo_ros 
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})


add_library(model_push SHARED src/model_push.cpp)
target_link_libraries(model_push ${GAZEBO_LIBRARIES})

add_library(gazebo_sticky_foot SHARED src/gazebo_sticky_foot.cpp)
target_link_libraries(gazebo_sticky_foot ${GAZEBO_LIBRARIES})

add_library(motor_plugin SHARED src/motor_plugin.cpp)
target_link_libraries(motor_plugin ${GAZEBO_LIBRARIES})
