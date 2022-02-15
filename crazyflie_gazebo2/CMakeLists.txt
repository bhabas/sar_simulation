cmake_minimum_required(VERSION 3.0.2)
project(crazyflie_gazebo2)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    gazebo_ros 
)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})


add_library(model_push SHARED src/model_push.cpp)
target_link_libraries(model_push ${GAZEBO_LIBRARIES})
