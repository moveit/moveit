# moveit/moveit:indigo-release
# Full debian-based install of MoveIt! using apt-get

FROM osrf/ros:indigo-desktop
MAINTAINER Dave Coleman dave@dav.ee

# apt-commands are combined in single RUN statement with "lists" folder removal to reduce image size
RUN apt-get update && \
    apt-get install -y \
        nano \
        ros-${ROS_DISTRO}-moveit-full && \
    rm -rf /var/lib/apt/lists/*
