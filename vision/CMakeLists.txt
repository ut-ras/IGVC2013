cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

FIND_PACKAGE(CUDA)
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_executable(vision src/vision_test.cpp)
rosbuild_add_executable(lounge src/vision_lounge.cpp)
#rosbuild_add_executable(decision src/vision_decision.cpp)
#rosbuild_add_executable(opencv_cuda_test src/opencv_cuda_test.cpp)
#rosbuild_add_executable(visualization src/visualizer.cpp)

rosbuild_add_executable(log_polar_processor nodes/log_polar_processor.cpp)
rosbuild_add_executable(grayscaler nodes/grayscaler.cpp)
rosbuild_add_executable(lane_detector nodes/lane_detector.cpp)
