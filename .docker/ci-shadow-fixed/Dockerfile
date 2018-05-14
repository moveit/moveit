# moveit/moveit:melodic-ci-shadow-fixed
# Sets up a base image to use for running Continuous Integration on Travis

FROM ubuntu:18.04
MAINTAINER Dave Coleman dave@picknik.ai

# TODO(davetcoleman): depend on ros:melodic-ros-base once OSRF makes
# this available (currently melodic is not released yet)
# https://github.com/osrf/docker_images/issues/134

# install packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list with shadow-fixed
RUN echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list

# hack to fix issue with tzdata install
# https://stackoverflow.com/questions/8671308/non-interactive-method-for-dpkg-reconfigure-tzdata
ENV DEBIAN_FRONTEND=noninteractive
ENV DEBCONF_NONINTERACTIVE_SEEN=true

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

ENV ROS_DISTRO melodic
ENV TERM xterm

# Setup catkin workspace
ENV CATKIN_WS=/root/ws_moveit
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
RUN wstool init . && \
    # Download moveit source so that we can get necessary dependencies
    wstool merge https://raw.githubusercontent.com/ros-planning/moveit/${ROS_DISTRO}-devel/moveit.rosinstall && \
    wstool update && \
    # Update apt-get because previous images clear this cache
    apt-get -qq update && \
    # Temp hack, see https://github.com/ros/ros_comm/issues/904
    #apt-get -qq remove -y ros-${ROS_DISTRO}-rostest && \
    # Do a dist-upgrade to ensure our CI is building on top of the latest version of packages
    apt-get -qq dist-upgrade && \
    # Install some base dependencies
    apt-get -qq install -y \
        # Some source builds require a package.xml be downloaded via wget from an external location
        wget \
        # Required for rosdep command
        sudo \
        # Preferred build tool
        python-catkin-tools && \
    # Download all dependencies of MoveIt!
    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    # Remove the source code from this container. TODO: in the future we may want to keep this here for further optimization of later containers
    cd .. && \
    rm -rf src/ && \
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*

# Continous Integration Setting
ENV IN_DOCKER 1
