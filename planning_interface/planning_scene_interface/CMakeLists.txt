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


# Eigen
find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
#add_definitions(${EIGEN_DEFINITIONS})
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(Threads REQUIRED)
find_package(Boost COMPONENTS
  python
  thread
  system
  REQUIRED
  )
find_package(PythonLibs REQUIRED)



# set(GEN_DIR_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/munged/${PROJECT_NAME}_bp)
# set(GEN_SHARED_LIBRARY_DIR ${GEN_DIR_ROOT}/bindings)
# set(GEN_PYTHON_ROOT ${GEN_DIR_ROOT}/conversions)
#set(GEN_DIR_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/munged)
#set(GEN_SHARED_LIBRARY_DIR ${GEN_DIR_ROOT})
#set(GEN_PYTHON_ROOT ${GEN_DIR_ROOT})

#file(WRITE ${GEN_DIR_ROOT}/__init__.py "\n")
#file(WRITE ${GEN_SHARED_LIBRARY_DIR}/__init__.py "\n")
#file(WRITE ${GEN_PYTHON_ROOT}/__init__.py "\n")

#add_definitions("-Wl,--no-undefined ")


# We need a special library for the ros time types since these are ROS builtins that don't have .msg files

# This macro generates the boost python bindings for the messages in a given ros package
#macro(bpmodule NAME)
#    include_directories(
#      ${PYTHON_INCLUDE_PATH}
#      ${Boost_INCLUDE_DIRS}
#    )
#
#    add_library(${NAME}_cpp SHARED ${ARGN})
#    target_link_libraries(${NAME}_cpp ${${PROJECT_NAME}_LIBRARIES})
#    
#    
#    set_target_properties(${NAME}_cpp
#      PROPERTIES
#      OUTPUT_NAME ${NAME}_cpp
#      LIBRARY_OUTPUT_DIRECTORY ${GEN_SHARED_LIBRARY_DIR}
#      PREFIX ""
#      LINKER_LANGUAGE CXX
#    )
#
#    target_link_libraries(${NAME}_cpp
#      ${Boost_LIBRARIES}
#      ${PYTHON_LIBRARIES}
#    )
#endmacro()

# Top level macro generates bindings as well as python conversion wrapper
#macro(message_gen_wrap ROS_PACKAGE)
#    set(GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/${ROS_PACKAGE}) #this is for generated cpp files.
#
#    execute_process(COMMAND /u/selliott/workspace_temp/boost_python_ros/scripts/generate_pkg_bindings.py
#      --package=${ROS_PACKAGE}
#      --cpp_target_dir=${GEN_DIR}
#      --py_target_dir=${GEN_PYTHON_ROOT}
#      --current_package=${PROJECT_NAME}
#    )
#
#    file(GLOB ${ROS_PACKAGE}_srcs  ${GEN_DIR}/*.cpp)
#
#    bpmodule(${ROS_PACKAGE}
#        ${${ROS_PACKAGE}_srcs}
#    )
#endmacro()


rosbuild_add_library(${PROJECT_NAME} src/planning_scene_interface.cpp)

find_package(PythonLibs)
include_directories (${PYTHON_INCLUDE_DIRS})
rosbuild_add_library(${PROJECT_NAME}_python src/planning_scene_interface.cpp)
target_link_libraries(${PROJECT_NAME}_python ${PROJECT_NAME} ${PYTHON_LIBRARIES})
set_target_properties(${PROJECT_NAME}_python PROPERTIES OUTPUT_NAME _planning_scene_interface PREFIX "")
rosbuild_link_boost(${PROJECT_NAME}_python thread python)


