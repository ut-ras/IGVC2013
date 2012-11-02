## CMakeLists.txt: cmake build definition file. Edit this file
## to your needs and then run "cmake ." in your project
## directory to build your makefiles. Once cmake is finished,
## run "make" to build your project.

## Sanity check; ignore this.
cmake_minimum_required(VERSION 2.6)

## Pull in a cmake module. This lets us use pkg-config, a handy
## utility you can use to query build information about
## installed libraries, within cmake.
include(${CMAKE_ROOT}/Modules/FindPkgConfig.cmake)

## Pull in Player material using pkg-config.
# find playerc, playerc++ compilation information
pkg_check_modules(PLAYERC "playerc")
pkg_check_modules(PLAYERCXX "playerc++")
# add playerc, playerc++ include directories and library paths
include_directories(${PLAYERC_INCLUDE_DIRS})
include_directories(${PLAYERCXX_INCLUDE_DIRS})
link_directories(${PLAYERC_LIBRARY_DIRS})
link_directories(${PLAYERCXX_LIBRARY_DIRS})

## Random cmake options
# Turn on for nice, colorful makefile output that may be
# annoying or affect your IDE's terminal emulator strangely.
set(CMAKE_COLOR_MAKEFILE off)

## Build flag definitions
# Release build flags. Add -OREALLY_FAST or
# -fignore-all-safety-checks, etc., here.  -O3 means optimize
# quite a bit. -DNDEBUG means define the preprocessor variable
# "NDEBUG". -Wall means print all possible warnings.
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -Wall")
# Debug build flags. -O0 means no optimizations. -g means
# include debugging information. -DDEBUG means define the
# preprocessor variable "DEBUG". -Wall means print all possible
# warnings. -Werror means issue a compilation error on a warning
# (instead of just printing a warning).
set(CMAKE_C_FLAGS_DEBUG "-O0 -g -DDEBUG -Wall -Werror")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -DDEBUG -Wall -Werror")

## Set build type
# "debug" for debugging; "release" for final thing
set(CMAKE_BUILD_TYPE debug)

## Add project directory to include path. Call this multiple
## times with different directories as arguments if you want to
## add other directories to your include path.
include_directories(".")

## Build libraries for each robot subsystem. Instead of
## declaring each library here, you can also have a
## CMakeLists.txt in each library's subdirectory and build
## recursively by putting suitable add_library(...) statements
## in each library's subdirectory and then replacing
## add_library(...) with add_subdirectory(subdirectory_name)
## here.
# top-down localization
add_library(localization
  udpsocket.cpp
  datagram.cpp
  objectposepacket.cpp
  packet.cpp
  localizableobject.cpp
  objectpose.cpp)

## add_executable(executable_file source1.cpp source2.cpp ...)

## Define any executables with add_executable. If you compiled
## independent libraries above, you only need to compile the
## file containing your main function and then link it against
## the libraries it needs.
##
## If you didn't build any libraries, you'll have to include
## every non-header source file your main file depends on in the
## call to add_executable.
# main robot control client
add_executable(client client.cpp)

## target_link_libraries(executable_file library1 library2 ...)

## Link executables with libraries with
## target_link_libraries. You may have your own libraries to
## link against, but you will definitely want to link against
## the Player libraries (${PLAYERC_LIBRARIES} and
## ${PLAYERCXX_LIBRARIES}) and the C math library ("m").

target_link_libraries(client
  localization
  m
  ${PLAYERC_LIBRARIES}
  ${PLAYERCXX_LIBRARIES})
