cmake_minimum_required(VERSION 3.0.2)
project(crazyflie_msgs)


find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CF_FlipData.msg
  CF_ImpactData.msg
  CF_MiscData.msg
  CF_StateData.msg
  RLCmd.msg
  RLData.msg
  CtrlData.msg
  PadConnect.msg
  RLConvg.msg
  MS.msg
  OF_SensorData.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  activateSticky.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  gazebo_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES CF_msgs
	CATKIN_DEPENDS gazebo_msgs geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
	${catkin_INCLUDE_DIRS}
)
