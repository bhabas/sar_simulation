cmake_minimum_required(VERSION 3.0.2)
project(example_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs    
    geometry_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CustomMessage.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)


include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

