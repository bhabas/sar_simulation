cmake_minimum_required(VERSION 3.0.2)
project(sar_gazebo)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  sar_msgs
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


add_library(Sticky_Foot_Plugin SHARED src/Sticky_Foot_Plugin.cpp)
target_link_libraries(Sticky_Foot_Plugin ${GAZEBO_LIBRARIES})

add_library(Motor_Plugin SHARED src/Motor_Plugin.cpp)
target_link_libraries(Motor_Plugin ${GAZEBO_LIBRARIES})

add_library(Cam_Update_Plugin SHARED src/Cam_Update_Plugin.cpp)
target_link_libraries(Cam_Update_Plugin ${GAZEBO_LIBRARIES})

add_library(DomainRand_plugin SHARED src/Domain_Randomization_Plugin.cpp)
target_link_libraries(DomainRand_plugin ${GAZEBO_LIBRARIES})

# EXTRA PLUGINS
add_library(ModelMove_plugin SHARED src/Model_Move_Plugin.cpp)
target_link_libraries(ModelMove_plugin ${GAZEBO_LIBRARIES})

add_library(Landing_Surface_Pose_plugin SHARED src/Landing_Surface_Pose_Plugin.cpp)
target_link_libraries(Landing_Surface_Pose_plugin ${GAZEBO_LIBRARIES})

add_library(Moment_plugin SHARED src/Model_Moment_Plugin.cpp)
target_link_libraries(Moment_plugin ${GAZEBO_LIBRARIES})