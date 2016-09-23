cmake_minimum_required(VERSION 2.8.12)
project(moveit_kinematics)
find_package(catkin REQUIRED)

add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  pluginlib
  actionlib
  roscpp
  rosconsole
  srdfdom
  urdf
  tf
  tf_conversions
)

find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN3 REQUIRED eigen3)

set(THIS_PACKAGE_INCLUDE_DIRS
    kdl_kinematics_plugin/include
    lma_kinematics_plugin/include
    srv_kinematics_plugin/include
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIRS}
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    pluginlib
    moveit_core
    moveit_ros_planning
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
)
include_directories(SYSTEM
                    ${EIGEN3_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
)

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(kdl_kinematics_plugin)
add_subdirectory(lma_kinematics_plugin)
add_subdirectory(srv_kinematics_plugin)
add_subdirectory(ikfast_kinematics_plugin)

install(
  FILES
    kdl_kinematics_plugin_description.xml
    lma_kinematics_plugin_description.xml
    srv_kinematics_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})
