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
  INCLUDE_DIRS plugins/include
  CATKIN_DEPENDS 
    roscpp 
    gazebo_ros 
)

include_directories(plugins/include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})


add_library(Sticky_Foot_Plugin SHARED plugins/src/Sticky_Foot_Plugin.cpp)
target_link_libraries(Sticky_Foot_Plugin ${GAZEBO_LIBRARIES})

add_library(Leg_Connect_Plugin SHARED plugins/src/Leg_Connect_Plugin.cpp)
target_link_libraries(Leg_Connect_Plugin ${GAZEBO_LIBRARIES})

add_library(Motor_Plugin SHARED plugins/src/Motor_Plugin.cpp)
target_link_libraries(Motor_Plugin ${GAZEBO_LIBRARIES})

add_library(Cam_Update_Plugin SHARED plugins/src/Cam_Update_Plugin.cpp)
target_link_libraries(Cam_Update_Plugin ${GAZEBO_LIBRARIES})

add_library(SAR_Update_Plugin SHARED plugins/src/SAR_Update_Plugin.cpp)
target_link_libraries(SAR_Update_Plugin ${GAZEBO_LIBRARIES})

add_library(Domain_Randomization_Plugin SHARED plugins/src/Domain_Randomization_Plugin.cpp)
target_link_libraries(Domain_Randomization_Plugin ${GAZEBO_LIBRARIES})

# EXTRA PLUGINS
add_library(Model_Move_Plugin SHARED plugins/src/Model_Move_Plugin.cpp)
target_link_libraries(Model_Move_Plugin ${GAZEBO_LIBRARIES})

add_library(Landing_Surface_Pose_Plugin SHARED plugins/src/Landing_Surface_Pose_Plugin.cpp)
target_link_libraries(Landing_Surface_Pose_Plugin ${GAZEBO_LIBRARIES})

add_library(Model_Moment_Plugin SHARED plugins/src/Model_Moment_Plugin.cpp)
target_link_libraries(Model_Moment_Plugin ${GAZEBO_LIBRARIES})