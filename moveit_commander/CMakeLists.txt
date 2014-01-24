cmake_minimum_required(VERSION 2.8.3)
project(moveit_commander)

find_package(catkin)

catkin_python_setup()

catkin_package()

install(DIRECTORY bin/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS)
