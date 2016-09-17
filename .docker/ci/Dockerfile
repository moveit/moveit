# moveit/moveit:jade-ci
# Sets up a base image to use for running Continuous Integration on Travis

FROM ros:jade-ros-base
MAINTAINER Dave Coleman dave@dav.ee

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
    # Install some base dependencies
    apt-get -qq install -y \
        # Required for rosdep command
        sudo \
        # Required for installing dependencies
        python-rosdep \
        # Preferred build tool
        python-catkin-tools \
        # Not sure if necessary:
        ros-$ROS_DISTRO-rosbash \
        ros-$ROS_DISTRO-rospack && \
    # Download all dependencies of MoveIt!
    rosdep update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    # Remove the source code from this container. TODO: in the future we may want to keep this here for further optimization of later containers
    cd .. && \
    rm -rf src/ && \
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*

# Continous Integration Setting
ENV IN_DOCKER 1
