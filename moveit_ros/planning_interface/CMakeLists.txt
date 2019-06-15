cmake_minimum_required(VERSION 2.8.12)
project(moveit_ros_planning_interface)

add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  moveit_msgs
  moveit_ros_planning
  moveit_ros_warehouse
  moveit_ros_manipulation
  moveit_ros_move_group
  eigen_conversions
  eigenpy
  tf_conversions
  tf
  roscpp
  actionlib
  rospy
  rosconsole
)

find_package(PythonInterp REQUIRED)
find_package(PythonLibs "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}" REQUIRED)

find_package(Boost REQUIRED)
if(Boost_VERSION LESS 106700)
  set(BOOST_PYTHON_COMPONENT python)
elseif()
  set(BOOST_PYTHON_COMPONENT python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR})
endif()

find_package(Boost REQUIRED COMPONENTS
  date_time
  filesystem
  program_options
  ${BOOST_PYTHON_COMPONENT}
  system
  thread
)
find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

set(THIS_PACKAGE_INCLUDE_DIRS
  py_bindings_tools/include
  common_planning_interface_objects/include
  planning_scene_interface/include
  move_group_interface/include
)

catkin_python_setup()

catkin_package(
  LIBRARIES
    moveit_common_planning_interface_objects
    moveit_planning_scene_interface
    moveit_move_group_interface
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    actionlib
    moveit_ros_planning
    moveit_ros_warehouse
    moveit_ros_manipulation
    moveit_ros_move_group
    moveit_msgs
  DEPENDS
    EIGEN3
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})

include_directories(SYSTEM
                    ${EIGEN3_INCLUDE_DIRS}
                    ${PYTHON_INCLUDE_DIRS})


add_subdirectory(py_bindings_tools)
add_subdirectory(common_planning_interface_objects)
add_subdirectory(planning_scene_interface)
add_subdirectory(move_group_interface)
add_subdirectory(robot_interface)
add_subdirectory(test)
