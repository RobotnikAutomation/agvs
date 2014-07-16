cmake_minimum_required(VERSION 2.8.3)
project(agvs_robot_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  robotnik_msgs
  roscpp
  sensor_msgs
  std_srvs
  std_msgs
  tf
  ackermann_msgs
)

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#   AckermannDrive.msg
#   AckermannDriveStamped.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(DEPENDENCIES agvs_robot_control std_msgs)


catkin_package(
  CATKIN_DEPENDS 
     roscpp
     robotnik_msgs 
     sensor_msgs
     std_srvs 
     std_msgs
     tf
     ackermann_msgs
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(agvs_robot_control src/agvs_robot_control.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(agvs_robot_control agvs_robot_control_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(agvs_robot_control
   ${catkin_LIBRARIES}
)
