cmake_minimum_required(VERSION 2.8.3)
project(moveit_ompl_planners_core)

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

find_package(Boost REQUIRED system filesystem date_time thread serialization)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin COMPONENTS
  moveit_core
  eigen_conversions
  REQUIRED)

include(cmake/addOMPLDep.cmake.in)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES
    moveit_ompl_interface
  DEPENDS
    moveit_core
  CFG_EXTRAS
    addOMPLDep.cmake)

include_directories(ompl_interface/include)
add_subdirectory(ompl_interface)
