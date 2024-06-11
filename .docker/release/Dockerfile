# Full debian-based install of MoveIt using apt-get

ARG ROS_DISTRO=noetic
FROM ros:${ROS_DISTRO}-ros-base
LABEL org.opencontainers.image.authors="Robert Haschke <rhaschke@techfak.uni-bielefeld.de>"

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
RUN apt-get update -q && \
    apt-get dist-upgrade -q -y && \
    apt-get install -y ros-${ROS_DISTRO}-moveit-* && \
    rm -rf /var/lib/apt/lists/*
