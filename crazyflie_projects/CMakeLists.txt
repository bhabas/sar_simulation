cmake_minimum_required(VERSION 3.0.2)
project(crazyflie_projects)


find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dashboard_gui_pkg
 CATKIN_DEPENDS rospy roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(example_test Policy_Mapping/Test_Script/example.cpp Policy_Mapping/Test_Script/nml.c Policy_Mapping/Test_Script/nml_util.c)
add_dependencies(example_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_test ${catkin_LIBRARIES})