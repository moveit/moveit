cmake_minimum_required(VERSION 2.8.3)
project(moveit_ompl_planners_ros_plugin)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(catkin COMPONENTS
  moveit_ros_planning
  moveit_ompl_planners_core
  roscpp
  rosconsole
  pluginlib
  tf
  dynamic_reconfigure
  REQUIRED)

generate_dynamic_reconfigure_options("ompl_interface_ros/cfg/OMPLDynamicReconfigure.cfg")

catkin_package(
  LIBRARIES
    moveit_ompl_interface_ros
  INCLUDE_DIRS
    ompl_interface_ros/include
  CATKIN_DEPENDS 
    moveit_ompl_planners_core
    moveit_ros_planning
    )

include_directories(SYSTEM
                    ${EIGEN_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})

include_directories(ompl_interface_ros/include 
                    ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_subdirectory(ompl_interface_ros)

install(FILES ompl_interface_ros_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
