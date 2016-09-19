# moveit/moveit:jade-release
# Full debian-based install of MoveIt! using apt-get

FROM moveit/moveit:jade-ci
MAINTAINER Dave Coleman dave@dav.ee

# Commands are combined in single RUN statement with "lists" folder removal to reduce image size
RUN apt-get update && \
    apt-get install -y \
        ros-${ROS_DISTRO}-moveit-* && \
    rm -rf /var/lib/apt/lists/*
