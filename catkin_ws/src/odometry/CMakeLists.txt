cmake_minimum_required(VERSION 2.8.2)
project(odometry)

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()

find_package(catkin REQUIRED roscpp rospy message_generation std_msgs nav_msgs geometry_msgs tf robairmain)

#find_package(boost REQUIRED) 

#add_message_files(DIRECTORY msg FILES )
#add_service_files(DIRECTORY srv FILES )

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()

#uncomment if you have defined services
#rosbuild_gensrv()



catkin_package(
CATKIN_DEPENDS message_runtime std_msgs nav_msgs geometry_msgs robairmain
) #here we can include dependecies like opencv

include_directories(${catkin_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
add_executable(odometry_node src/odometry.cpp src/interface.cpp)
add_dependencies(odometry_node robairmain_generate_messages_cpp)
target_link_libraries(odometry_node ${catkin_LIBRARIES})

add_executable(odometry_tf_broadcaster src/odometry_tf_broadcaster.cpp)
target_link_libraries(odometry_tf_broadcaster ${catkin_LIBRARIES})
