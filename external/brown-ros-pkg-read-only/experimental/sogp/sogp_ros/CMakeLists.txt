cmake_minimum_required(VERSION 2.4.6)

rosbuild_add_executable(sogp_regress sogp_ros.cpp)
rosbuild_add_executable(sogp_test sogp_test.cpp)
TARGET_LINK_LIBRARIES(sogp_regress sogp newmat)
