#!/bin/sh
export ROS_DISTRO=melodic
export ROS_REPO=ros
export TRAVIS_BRANCH=master
export TEST=clang-tidy-fix
export TEST_BLACKLIST="moveit_jog_arm \
moveit_setup_assistant \
moveit_ros_planning_interface \
moveit_ros_robot_interaction \
moveit_core \
moveit_ros_perception \
moveit_kinematics \
moveit_ros_visualization \
moveit_commander"
export UPSTREAM_WORKSPACE=moveit.rosinstall
export WARNINGS_OK=false
export CC=clang
export CC_FOR_BUILD=clang
export CXX=clang++
export CXX_FOR_BUILD=clang++
export CXXFLAGS="-Wall -Wextra -Wwrite-strings -Wunreachable-code -Wpointer-arith -Wredundant-decls -Wno-overloaded-virtual"