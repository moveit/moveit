#!/bin/bash

echo "run_before script"
apt-get -qq remove libfcl0 libfcl-dev ros-kinetic-octomap
cp moveit_kinetic_cpp11/package.xml fcl/package.xml

# Remove package.xml otherwise it will think there are multiple packages
rm moveit_kinetic_cpp11/package.xml
