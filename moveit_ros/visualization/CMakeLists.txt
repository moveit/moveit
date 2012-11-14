cmake_minimum_required(VERSION 2.8.3)
project(moveit_ros_visualization)

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

find_package(Boost REQUIRED thread date_time system filesystem)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

# Ogre
pkg_check_modules(OGRE OGRE)
include_directories(SYSTEM ${OGRE_INCLUDE_DIRS} )
link_directories( ${OGRE_LIBRARY_DIRS} )

# Qt Stuff
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)
include_directories(${QT_INCLUDE_DIR})

find_package(catkin COMPONENTS
  moveit_ros_planning_interface
  moveit_ros_warehouse
  eigen_conversions
  geometric_shapes
  interactive_markers
  class_loader
  rviz
  tf
  roscpp
  rosconsole
  REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES
    moveit_rviz_plugin_core
  INCLUDE_DIRS
    rviz_plugin/include
  DEPENDS
    moveit_ros_planning_interface
    )

include_directories(rviz_plugin/include)

add_subdirectory(rviz_plugin)

install(FILES moveit_rviz_plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
