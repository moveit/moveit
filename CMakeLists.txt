cmake_minimum_required(VERSION 2.8.3)
project(moveit_resources)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
