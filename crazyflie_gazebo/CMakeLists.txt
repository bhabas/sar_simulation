cmake_minimum_required(VERSION 3.0.2)
project(crazyflie_gazebo)


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


add_library(gazebo_sticky_foot SHARED src/gazebo_sticky_foot.cpp)
target_link_libraries(gazebo_sticky_foot ${GAZEBO_LIBRARIES})

add_library(motor_plugin SHARED src/motor_plugin.cpp)
target_link_libraries(motor_plugin ${GAZEBO_LIBRARIES})

add_library(OpticalFlow_plugin SHARED src/OpticalFlow_plugin.cpp)
target_link_libraries(OpticalFlow_plugin ${GAZEBO_LIBRARIES})

add_library(Moment_plugin SHARED src/model_Moment.cpp)
target_link_libraries(Moment_plugin ${GAZEBO_LIBRARIES})

add_library(DomainRand_plugin SHARED src/DomainRandomization.cpp)
target_link_libraries(DomainRand_plugin ${GAZEBO_LIBRARIES})
