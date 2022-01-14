#!/bin/bash

# Script to test ikfast plugin creation and functionality

# We will create ikfast plugins for fanuc and panda from moveit_resources
# using the script auto_create_ikfast_moveit_plugin.sh

set -e # fail script on error

sudo update-alternatives --install /usr/bin/python python /usr/bin/python2 1
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 2
sudo apt-get -q update

if [ "$ROS_DISTRO" == "noetic" ]; then
	sudo update-alternatives --set python /usr/bin/python3
	sudo apt-get -qq install python3-lxml python3-yaml
else
	sudo update-alternatives --set python /usr/bin/python2
	sudo apt-get -qq install python-lxml python-yaml
fi

# Clone moveit_resources for URDFs. They are not available before running docker.
git clone -q --depth=1 https://github.com/ros-planning/moveit_resources /tmp/resources
fanuc=/tmp/resources/fanuc_description/urdf/fanuc.urdf
panda=/tmp/resources/panda_description/urdf/panda.urdf

export QUIET=${QUIET:=1}

# Create ikfast plugins for Fanuc and Panda
moveit_kinematics/ikfast_kinematics_plugin/scripts/auto_create_ikfast_moveit_plugin.sh \
	--name fanuc --pkg $PWD/fanuc_ikfast_plugin $fanuc manipulator base_link tool0

moveit_kinematics/ikfast_kinematics_plugin/scripts/auto_create_ikfast_moveit_plugin.sh \
	--name panda --pkg $PWD/panda_ikfast_plugin $panda panda_arm panda_link0 panda_link8

echo "Done."
