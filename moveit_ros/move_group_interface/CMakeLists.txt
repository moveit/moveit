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

rosbuild_add_boost_directories()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/move_group.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(move_group_cmd src/move_group_cmdline.cpp)
target_link_libraries(move_group_cmd ${PROJECT_NAME})

find_package(PythonLibs)
include_directories (${PYTHON_INCLUDE_DIRS})

rosbuild_add_library(${PROJECT_NAME}_python src/wrap_python_move_group.cpp)
target_link_libraries(${PROJECT_NAME}_python ${PROJECT_NAME} ${PYTHON_LIBRARIES})
set_target_properties(${PROJECT_NAME}_python PROPERTIES OUTPUT_NAME _move_group PREFIX "")
rosbuild_link_boost(${PROJECT_NAME}_python thread python)
