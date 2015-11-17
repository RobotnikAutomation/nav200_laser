cmake_minimum_required(VERSION 2.8.3)
project(nav200_laser)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  std_msgs
  roscpp
  tf
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Pose2D_nav.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
 INCLUDE_DIRS include
 # LIBRARIES nav200_laser
 #CATKIN_DEPENDS roscpp
 # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(node_position src/node_position.cpp src/SerialDevice.cc)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(node_position nav200_laser_generate_messages_cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(node_position
   ${catkin_LIBRARIES}
 )

