# moveit/moveit:kinetic-source
# Downloads the moveit source code, install remaining debian dependencies, and builds workspace

FROM moveit/moveit:kinetic-ci-shadow-fixed
MAINTAINER Dave Coleman dave@dav.ee

ENV CATKIN_WS=/root/ws_moveit
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Download moveit source
RUN wstool init . && \
    wstool merge https://raw.githubusercontent.com/ros-planning/moveit/${ROS_DISTRO}-devel/moveit.rosinstall && \
    wstool update

# Update apt-get because osrf image clears this cache. download deps
# Note that because we're building on top of kinetic-ci, there should not be any deps installed
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
RUN catkin config --extend /opt/ros/$ROS_DISTRO --install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Status rate is limited so that just enough info is shown to keep Docker from timing out, but not too much
    # such that the Docker log gets too long (another form of timeout)
    catkin build --jobs 1 --limit-status-rate 0.001 --no-notify
