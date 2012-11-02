#this is just a basic CMakeLists.txt, for more information see the cmake manpage
cmake_minimum_required(VERSION 2.4.6)

#for testing the shared library you probably need some test app too
rosbuild_add_executable(sogptest SOGPtest.cc)

#need to link to some other libraries ? just add them here
TARGET_LINK_LIBRARIES(sogptest sogp newmat)

