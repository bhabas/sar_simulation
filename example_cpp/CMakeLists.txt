cmake_minimum_required(VERSION 3.0.2)
project(example_cpp)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  example_msgs ## INCLUDE PACKAGE WITH CUSTOM ROS MESSAGES
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES 
 CATKIN_DEPENDS roscpp 
#  DEPENDS 
)

include_directories(include ${catkin_INCLUDE_DIRS})


## CREATE C++ EXECUTABLE
add_executable(pub_example src/pub_example.cpp src/nml.c src/nml_util.c)
add_dependencies(pub_example ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub_example ${catkin_LIBRARIES})