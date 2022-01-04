# moveit/moveit:noetic-ci-shadow-fixed
# Sets up a base image to use for running Continuous Integration on Travis

ARG IMAGE=noetic
FROM moveit/moveit:${IMAGE}-ci
MAINTAINER Robert Haschke rhaschke@techfak.uni-bielefeld.de

# Switch to ros-shadow-fixed
RUN echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -cs` main" | tee /etc/apt/sources.list.d/ros-latest.list

# Upgrade packages to ros-shadow-fixed and clean apt-cache within one RUN command
RUN apt-get -qq update && \
    apt-get -qq dist-upgrade && \
    #
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*
