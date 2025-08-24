cmake_minimum_required(VERSION 3.16)
project(moveit_commander)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(
  PROGRAMS bin/${PROJECT_NAME}_cmdline.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_subdirectory(test)
