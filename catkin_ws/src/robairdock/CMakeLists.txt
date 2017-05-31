cmake_minimum_required(VERSION 2.8.3)
project(robairdock)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  genmsg
  std_msgs
  geometry_msgs
  message_generation
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories( ${catkin_INCLUDE_DIRS})
