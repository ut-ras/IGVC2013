cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

SET(CMAKE_VERBOSE_MAKEFILE ON)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#include newmat header directory
INCLUDE_DIRECTORIES(3rdparty/newmat)
INCLUDE_DIRECTORIES(sogp)

#Generate message stubs
rosbuild_genmsg()
#Generate service stubs
rosbuild_gensrv()

#Build directories in the following order
add_subdirectory(3rdparty/newmat)
add_subdirectory(sogp)
add_subdirectory(utils)
add_subdirectory(sogp_ros)
