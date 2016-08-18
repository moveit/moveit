# moveit/moveit:jade-source
# Downloads the moveit source code, install remaining debian dependencies, and builds workspace

FROM osrf/ros:jade-desktop
MAINTAINER Dave Coleman dave@dav.ee

ENV CATKIN_WS=/root/ws_moveit
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Download moveit source
RUN wstool init . && \
    wstool merge https://raw.githubusercontent.com/ros-planning/moveit/${ROS_DISTRO}-devel/moveit.rosinstall && \
    wstool update

# Update apt-get because osrf image clears this cache. download deps
RUN echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -cs` main" | tee --append /etc/apt/sources.list.d/ros-latest.list && \
    rosdep update && \
    apt-get -qq update && \
    apt-get -qq install -y \
        python-catkin-tools  \
        less \
        ssh \
        emacs \
        git-core \
        bash-completion \
        tree \
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
