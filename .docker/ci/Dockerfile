# moveit/moveit:noetic-ci
# Sets up a base image to use for running Continuous Integration on Travis

FROM ros:noetic-ros-base
MAINTAINER Dave Coleman dave@picknik.ai

ENV TERM xterm

# Setup (temporary) ROS workspace
WORKDIR /root/ws_moveit

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
RUN \
    # Update apt package list as previous containers clear the cache
    apt-get -qq update && \
    apt-get -qq dist-upgrade && \
    #
    # Install some base dependencies
    apt-get -qq install --no-install-recommends -y \
        # Some basic requirements
        wget git sudo \
        # Preferred build tools
        python3-catkin-tools python3-osrf-pycommon \
        clang clang-format-10 clang-tidy clang-tools \
        ccache && \
    #
    # Download MoveIt source, so that we can fetch all necessary dependencies
    wstool init --shallow src https://raw.githubusercontent.com/ros-planning/moveit/${ROS_DISTRO}-devel/moveit.rosinstall && \
    #
    # Download all dependencies of MoveIt
    rosdep update && \
    DEBIAN_FRONTEND=noninteractive \
    rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    # Remove the source code from this container
    rm -rf src && \
    #
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*

# Continous Integration Setting
ENV IN_DOCKER 1
