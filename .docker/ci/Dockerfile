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
        wget git sudo python3-pip \
        # Preferred build tools
        clang clang-format-10 clang-tidy clang-tools \
        ccache && \
    # Manually install catkin-tools from latest HEAD (TODO: remove if re-released)
    pip3 install git+https://github.com/catkin/catkin_tools.git && \
    #
    # Build and install not yet released dependencies (TODO: remove if released)
    wstool init --shallow src https://raw.githubusercontent.com/ros-planning/moveit/${ROS_DISTRO}-devel/noetic.rosinstall && \
    # We only depend on franka_description, other franka packages require even more dependencies...
    mv src/franka_ros/franka_description src/ && rm -rf src/franka_ros && \
    rosdep update && \
    DEBIAN_FRONTEND=noninteractive \
    rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
	 apt-get -qq install -y ros-noetic-tf2-eigen ros-noetic-tf2-kdl ros-noetic-tf2-geometry-msgs && \ 
    catkin config --extend /opt/ros/noetic --install --install-space /opt/ros/noetic && \
    catkin build && \
    rm src/.rosinstall && \
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
    # Cleanup full workspace (TODO: remove, if dependencies are released)
    rm -rf src logs build devel .catkin_tools && \
    #
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*

# Continous Integration Setting
ENV IN_DOCKER 1
