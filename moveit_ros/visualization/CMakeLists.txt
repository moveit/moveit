cmake_minimum_required(VERSION 2.8.12)
project(moveit_ros_visualization)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread date_time system filesystem)

find_package(PkgConfig REQUIRED)

# Ogre
pkg_check_modules(OGRE OGRE)
link_directories( ${OGRE_LIBRARY_DIRS} )

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  moveit_ros_robot_interaction
  moveit_ros_warehouse
  moveit_ros_perception
  eigen_conversions
  geometric_shapes
  interactive_markers
  class_loader
  rviz
  tf
  roscpp
  rosconsole
  object_recognition_msgs
  rostest
)
pkg_search_module(EIGEN3 REQUIRED eigen3)

# Qt Stuff
if(rviz_QT_VERSION VERSION_LESS "5")
	find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
	include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
else()
	find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
	set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  LIBRARIES
    moveit_rviz_plugin_render_tools
    moveit_robot_state_rviz_plugin_core
    moveit_motion_planning_rviz_plugin_core
    moveit_trajectory_rviz_plugin_core
    moveit_planning_scene_rviz_plugin_core
  INCLUDE_DIRS
    rviz_plugin_render_tools/include
    robot_state_rviz_plugin/include
    planning_scene_rviz_plugin/include
    motion_planning_rviz_plugin/include
    trajectory_rviz_plugin/include
    ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_ros_planning_interface
    moveit_ros_robot_interaction
    )

catkin_install_python(PROGRAMS scripts/moveit_joy.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_python_setup()

include_directories(rviz_plugin_render_tools/include
                    robot_state_rviz_plugin/include
                    planning_scene_rviz_plugin/include
                    motion_planning_rviz_plugin/include
                    trajectory_rviz_plugin/include
		    ${catkin_INCLUDE_DIRS})

include_directories(SYSTEM
                    ${EIGEN3_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${QT_INCLUDE_DIR}
                    ${OGRE_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(rviz_plugin_render_tools)
add_subdirectory(robot_state_rviz_plugin)
add_subdirectory(planning_scene_rviz_plugin)
add_subdirectory(motion_planning_rviz_plugin)
add_subdirectory(trajectory_rviz_plugin)

install(FILES
  motion_planning_rviz_plugin_description.xml
  trajectory_rviz_plugin_description.xml
  planning_scene_rviz_plugin_description.xml
  robot_state_rviz_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest)
  add_rostest(test/moveit_joy.test)
endif()
