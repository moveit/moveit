# moveit/moveit:melodic-release
# Full debian-based install of MoveIt! using apt-get

FROM moveit/moveit:melodic-ci
MAINTAINER Dave Coleman dave@picknik.ai

# Commands are combined in single RUN statement with "lists" folder removal to reduce image size
RUN apt-get update && \
    apt-get install -y \
        ros-${ROS_DISTRO}-moveit-* && \
    rm -rf /var/lib/apt/lists/*
