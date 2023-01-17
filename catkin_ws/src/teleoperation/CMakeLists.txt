cmake_minimum_required(VERSION 2.8.2)
project(teleoperation)

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()

find_package(catkin REQUIRED roscpp rospy message_generation std_msgs nav_msgs geometry_msgs tf)

#find_package(boost REQUIRED) 

#add_message_files(DIRECTORY msg FILES Status.msg)
#add_service_files(DIRECTORY srv FILES )

generate_messages(DEPENDENCIES std_msgs nav_msgs geometry_msgs)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()

#uncomment if you have defined services
#rosbuild_gensrv()

catkin_package(
CATKIN_DEPENDS message_runtime std_msgs nav_msgs geometry_msgs
) #here we can include dependecies like opencv

include_directories(include ${catkin_INCLUDE_DIRS})

install (DIRECTORY scripts/
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	USE_SOURCE_PERMISSIONS)

#common commands for building c++ executables and libraries
#add_library(libwifibot src/libwifibot.cpp)
#target_link_libraries(libwifibot ${catkin_LIBRARIES})
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#add_executable(wifibot_node src/wifibot.cpp)
#target_link_libraries(wifibot_node libwifibot ${catkin_LIBRARIES})

#add_executable(wifibot_tf_broadcaster src/wifibot_tf_broadcaster.cpp)
#target_link_libraries(wifibot_tf_broadcaster ${catkin_LIBRARIES})
