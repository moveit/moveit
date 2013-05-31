cmake_minimum_required(VERSION 2.8.3)
project(moveit_resources)

find_package(catkin REQUIRED)

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/include")
catkin_package(INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/include)

set(MOVEIT_TEST_RESOURCES_DIR_INSTALL "${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test")
set(MOVEIT_TEST_RESOURCES_DIR_LOCAL "${CMAKE_CURRENT_SOURCE_DIR}/test")

configure_file("include/moveit/test_resources/config.h.in" "include/moveit/test_resources/config.h")
configure_file("include/moveit/test_resources/config_install.h.in" "installconfig/config_install.h")

install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/installconfig/config_install.h"
  DESTINATION "include/moveit/test_resources"
  RENAME "config.h")
