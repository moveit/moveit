# moveit/moveit:kinetic-ci-wily
# Sets up a base image to use for running Continuous Integration on Travis for Ubuntu Wily

FROM osrf/ubuntu_32bit:wily
MAINTAINER Dave Coleman dave@dav.ee

# ----------------------------------------------------------------------------------
# From https://github.com/osrf/docker_images/

# setup environment
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

# setup keys
RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu wily main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        python-rosdep \
        python-rosinstall \
        python-vcstools && \
    rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
    rosdep update

ENV ROS_DISTRO kinetic

# ----------------------------------------------------------------------------------
# Standard MoveIt! Docker

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

CMD ["bash"]
