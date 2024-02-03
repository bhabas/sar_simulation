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

add_library(Step_World_Plugin SHARED plugins/src/Step_World_Plugin.cpp)
target_link_libraries(Step_World_Plugin ${GAZEBO_LIBRARIES})

add_library(SAR_Update_Plugin SHARED plugins/src/SAR_Update_Plugin.cpp)
target_link_libraries(SAR_Update_Plugin ${GAZEBO_LIBRARIES})

add_library(Sticky_Leg_Plugin SHARED plugins/src/Sticky_Leg_Plugin.cpp)
target_link_libraries(Sticky_Leg_Plugin ${GAZEBO_LIBRARIES})

add_library(Motor_Plugin SHARED plugins/src/Motor_Plugin.cpp)
target_link_libraries(Motor_Plugin ${GAZEBO_LIBRARIES})

add_library(Landing_Surface_Pose_Plugin SHARED plugins/src/Landing_Surface_Pose_Plugin.cpp)
target_link_libraries(Landing_Surface_Pose_Plugin ${GAZEBO_LIBRARIES})

#add_library(Example_Cam_Plugin_Ref SHARED plugins/src/Example_Cam_Plugin_Ref.cpp)
#target_link_libraries(Example_Cam_Plugin_Ref ${GAZEBO_LIBRARIES})


# EXTRA PLUGINS
add_library(Model_Move_Plugin SHARED plugins/src/Model_Move_Plugin.cpp)
target_link_libraries(Model_Move_Plugin ${GAZEBO_LIBRARIES})

add_library(Model_Moment_Plugin SHARED plugins/src/Model_Moment_Plugin.cpp)
target_link_libraries(Model_Moment_Plugin ${GAZEBO_LIBRARIES})