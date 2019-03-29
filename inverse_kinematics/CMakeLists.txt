cmake_minimum_required(VERSION 2.8.3)
project(inverse_kinematics)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
)

add_message_files(
  FILES
  info_kinematics_msg.msg
  inverse_kinematics_coordinate_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES inverse_kinematics
  CATKIN_DEPENDS roscpp rospy  std_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(inverse
src/Serial.cpp
src/Play_Kinematics.cpp
src/pid_control_float.cpp
src/Moving_point.cpp
src/inverse_kinematics.cpp
)

add_executable(inverse_kinematics src/inverse_kinematics.cpp)

target_link_libraries(inverse_kinematics
  ${catkin_LIBRARIES}
  inverse
)





