cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

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
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


link_directories(/usr/local/lib)

 
rosbuild_add_executable(localize_republish src/localize_republish.cpp src/datagram.cpp src/localizableobject.cpp src/objectpose.cpp src/objectposepacket.cpp src/packet.cpp src/udpsocket.cpp)

#rosbuild_add_executable(randomicreate src/randomwalker.cpp)

#rosbuild_add_executable(rlglue_icreate src/rlglue_icreate.cpp src/datagram.cpp src/localizableobject.cpp src/objectpose.cpp src/objectposepacket.cpp src/packet.cpp src/udpsocket.cpp)

#target_link_libraries(rlglue_icreate rlutils rlenvironment rlgluenetdev)

rosbuild_add_executable(rlglue_randomexperiment src/rlglue_randomexperiment.cpp)

target_link_libraries(rlglue_randomexperiment rlutils rlexperiment rlgluenetdev)

rosbuild_add_executable(rlglue_randomwalker src/rlglue_randomwalker.cpp)

target_link_libraries(rlglue_randomwalker rlutils rlagent rlgluenetdev)

#rosbuild_add_executable(randomnumber_tester src/randomnumber_tester.cpp)
