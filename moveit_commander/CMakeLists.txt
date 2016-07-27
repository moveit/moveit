cmake_minimum_required(VERSION 2.8.3)
project(moveit_commander)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(PROGRAMS bin/${PROJECT_NAME}_cmdline.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
