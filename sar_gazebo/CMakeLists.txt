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

add_library(Domain_Randomization_Plugin SHARED src/Domain_Randomization_Plugin.cpp)
target_link_libraries(Domain_Randomization_Plugin ${GAZEBO_LIBRARIES})

# EXTRA PLUGINS
add_library(Model_Move_Plugin SHARED src/Model_Move_Plugin.cpp)
target_link_libraries(Model_Move_Plugin ${GAZEBO_LIBRARIES})

add_library(Landing_Surface_Pose_Plugin SHARED src/Landing_Surface_Pose_Plugin.cpp)
target_link_libraries(Landing_Surface_Pose_Plugin ${GAZEBO_LIBRARIES})

add_library(Model_Moment_Plugin SHARED src/Model_Moment_Plugin.cpp)
target_link_libraries(Model_Moment_Plugin ${GAZEBO_LIBRARIES})