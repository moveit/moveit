cmake_minimum_required(VERSION 2.8.3)
project(moveit_ompl_planners_core)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  eigen_conversions
  )

include(cmake/addOMPLDep.cmake.in)


catkin_package(
  LIBRARIES
    moveit_ompl_interface
  INCLUDE_DIRS
    ompl_interface/include
  DEPENDS
    moveit_core
  CFG_EXTRAS
    addOMPLDep.cmake)

include_directories(ompl_interface/include 
                    ${catkin_INCLUDE_DIRS} 
                    ${EIGEN_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})


add_subdirectory(ompl_interface)
