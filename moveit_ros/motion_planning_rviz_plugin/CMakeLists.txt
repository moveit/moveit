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

set(SOURCE_FILES src/planning_display.cpp src/init.cpp)

include(CMakeDetermineSystem)
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  rosbuild_add_library(${PROJECT_NAME} MODULE ${SOURCE_FILES})
else(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  rosbuild_add_library(${PROJECT_NAME} ${SOURCE_FILES})
endif(CMAKE_SYSTEM_NAME MATCHES "Darwin")

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )
                                    
target_link_libraries(${PROJECT_NAME} ${wxWidgets_LIBRARIES} ${OGRE_LIBRARIES})
rosbuild_link_boost(${PROJECT_NAME} thread)
