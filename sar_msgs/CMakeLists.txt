cmake_minimum_required(VERSION 3.0.2)
project(sar_msgs)


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
  SAR_TriggerData.msg
  SAR_ImpactData.msg
  SAR_MiscData.msg
  SAR_StateData.msg
  RL_Data.msg
  RL_History.msg
  CTRL_Data.msg
  CTRL_Debug.msg
  CTRL_Cmd.msg
  Sticky_Pad_Connect.msg
  MS.msg
  GenericLogData.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  Activate_Sticky_Pads.srv
  Logging_CMD.srv

  Inertia_Params.srv
  Hinge_Params.srv
  Camera_Params.srv
  Surface_Params.srv
  
  CTRL_Cmd_srv.srv
  CTRL_Get_Obs.srv
  Model_Move.srv
  Cam_Settings.srv
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
