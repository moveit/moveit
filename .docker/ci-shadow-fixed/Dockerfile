# moveit/moveit:indigo-ci-shadow-fixed
# Sets up a base image to use for running Continuous Integration on Travis

FROM moveit/moveit:indigo-ci
MAINTAINER Dave Coleman dave@dav.ee

ENV TERM xterm

# Setup catkin workspace
ENV CATKIN_WS=/root/ws_moveit
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# Switch to shadow-fixed
RUN echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -cs` main" | tee /etc/apt/sources.list.d/ros-latest.list && \
    # Update apt-get because previous images clear this cache
    apt-get -qq update && \
    # Do a dist-upgrade to ensure our CI is building on top of the latest version of packages
    apt-get -qq dist-upgrade && \
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*

# Continous Integration Setting
ENV IN_DOCKER 1
