cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Release)
rosbuild_init()

execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.ompl
                RESULT_VARIABLE _make_failed)
if(_make_failed)
  message(FATAL_ERROR "Build of ompl failed")
endif(_make_failed)

message("Using ompl from ${OMPL_SOURCE}")

set(PROJECT_SOURCE_DIR ${PROJECT_SOURCE_DIR}/${OMPL_SOURCE})

option(OMPL_IN_ROS "Build OMPL against ROS" ON)
option(OMPL_BUILD_DEMOS "Build OMPL demos" OFF)
option(OMPL_BUILD_PYBINDINGS "Build OMPL Python bindings" OFF)
option(OMPL_BUILD_DOC "Build OMPL documentation" OFF)

add_subdirectory(${OMPL_SOURCE})
