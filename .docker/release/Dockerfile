# moveit/moveit:jade-release
# Full debian-based install of MoveIt! using apt-get

FROM osrf/ros:jade-desktop
MAINTAINER Dave Coleman dave@dav.ee

# apt-commands are combined in single RUN statement with "lists" folder removal to reduce image size
RUN echo 'deb http://packages.ros.org/ros-shadow-fixed/ubuntu xenial main' | tee --append /etc/apt/sources.list.d/ros-latest.list && \
    apt-get update && \
    apt-get install -y \
        nano \
        ros-${ROS_DISTRO}-moveit-* && \
    rm -rf /var/lib/apt/lists/*
