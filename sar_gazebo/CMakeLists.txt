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


add_library(gazebo_sticky_foot SHARED src/gazebo_sticky_foot.cpp)
target_link_libraries(gazebo_sticky_foot ${GAZEBO_LIBRARIES})

add_library(motor_plugin SHARED src/motor_plugin.cpp)
target_link_libraries(motor_plugin ${GAZEBO_LIBRARIES})

add_library(Cam_Update_Plugin SHARED src/Cam_Update_Plugin.cpp)
target_link_libraries(Cam_Update_Plugin ${GAZEBO_LIBRARIES})

add_library(DomainRand_plugin SHARED src/DomainRandomization.cpp)
target_link_libraries(DomainRand_plugin ${GAZEBO_LIBRARIES})

# EXTRA PLUGINS
add_library(ModelMove_plugin SHARED src/Model_Move.cpp)
target_link_libraries(ModelMove_plugin ${GAZEBO_LIBRARIES})

add_library(Landing_Surface_Pose_plugin SHARED src/Landing_Surface_Pose.cpp)
target_link_libraries(Landing_Surface_Pose_plugin ${GAZEBO_LIBRARIES})

add_library(Moment_plugin SHARED src/model_Moment.cpp)
target_link_libraries(Moment_plugin ${GAZEBO_LIBRARIES})