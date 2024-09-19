cmake_minimum_required(VERSION 3.0.2)
project(sar_lidar)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sar_msgs
  pcl_ros
  pcl_conversions
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp rospy pcl_ros pcl_conversions
#  DEPENDS system_lib
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(lidar src/SAR_Lidar.cpp)
add_dependencies(lidar ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar ${catkin_LIBRARIES} ${CURSES_LIBRARIES})


add_executable(visualize_bounding_boxes src/visualize_bounding_boxes.cpp)
add_dependencies(visualize_bounding_boxes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualize_bounding_boxes
  ${catkin_LIBRARIES}
)

add_executable(visualize_bounding_boxes_camera src/visualize_bounding_boxes_camera.cpp)
add_dependencies(visualize_bounding_boxes_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualize_bounding_boxes_camera
  ${catkin_LIBRARIES}
)



