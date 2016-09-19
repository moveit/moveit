# moveit/moveit:indigo-source
# Downloads the moveit source code, install remaining debian dependencies, and builds workspace

FROM moveit/moveit:indigo-ci
MAINTAINER Dave Coleman dave@dav.ee

ENV CATKIN_WS=/root/ws_moveit
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Download moveit source
RUN wstool init . && \
    wstool merge https://raw.githubusercontent.com/ros-planning/moveit/${ROS_DISTRO}-devel/moveit.rosinstall && \
    wstool update

# Update apt-get because osrf image clears this cache. download deps
# Note that because we're building on top of indigo-ci, there should not be any deps installed
# unless something has changed in the source code since the other container was made
# (they are triggered together so should only be one-build out of sync)
RUN apt-get -qq update && \
    apt-get -qq install -y \
        wget && \
    rosdep update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    rm -rf /var/lib/apt/lists/*

# Replacing shell with bash for later docker build commands
RUN mv /bin/sh /bin/sh-old && \
    ln -s /bin/bash /bin/sh

# Build repo
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
RUN source /ros_entrypoint.sh && \
    catkin build --no-status
