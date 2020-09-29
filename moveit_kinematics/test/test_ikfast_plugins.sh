#!/bin/bash

# Script to test ikfast plugin creation and functionality
# This script is intended to run as a BEFORE_DOCKER_SCRIPT from Travis.
# Particularly we assume that the travis_* utility functions are available.

# We will create ikfast plugins for fanuc and panda from moveit_resources
# using the script auto_create_ikfast_moveit_plugin.sh

sudo update-alternatives --install /usr/bin/python python /usr/bin/python2 1
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 2

if [ "$ROS_DISTRO" == "noetic" ]; then
	sudo update-alternatives --set python /usr/bin/python3
	travis_run sudo apt-get -qq install python3-lxml python3-yaml
else
	sudo update-alternatives --set python /usr/bin/python2
	travis_run sudo apt-get -qq install python-lxml python-yaml
fi

# Clone moveit_resources for URDFs. They are not available before running docker.
travis_run git clone -q --depth=1 https://github.com/ros-planning/moveit_resources /tmp/resources
fanuc=/tmp/resources/fanuc_description/urdf/fanuc.urdf
panda=/tmp/resources/panda_description/urdf/panda.urdf

# Translate environment variable QUIET=[0 | 1] into actual option
test ${QUIET:-1} -eq 0 && QUIET="" || QUIET="--quiet"

# Create ikfast plugins for Fanuc and Panda
travis_run moveit_kinematics/ikfast_kinematics_plugin/scripts/auto_create_ikfast_moveit_plugin.sh \
	$QUIET --name fanuc --pkg $PWD/fanuc_ikfast_plugin $fanuc manipulator base_link tool0

travis_run moveit_kinematics/ikfast_kinematics_plugin/scripts/auto_create_ikfast_moveit_plugin.sh \
	$QUIET --name panda --pkg $PWD/panda_ikfast_plugin $panda panda_arm panda_link0 panda_link8

echo "Done."
