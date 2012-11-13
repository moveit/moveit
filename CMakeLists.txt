cmake_minimum_required(VERSION 2.8.3)
project(moveit_commander)

find_package(catkin COMPONENTS
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_manipulation
  moveit_ros_warehouse
)

catkin_package(
  DEPENDS
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_manipulation
    moveit_ros_warehouse
    )

install(DIRECTORY src/moveit_commander DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
install(PROGRAMS scripts/moveit_commander_cmdline.py DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
