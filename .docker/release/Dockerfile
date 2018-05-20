# moveit/moveit:melodic-release
# Full debian-based install of MoveIt! using apt-get

FROM ros:melodic-ros-base
MAINTAINER Dave Coleman dave@picknik.ai

RUN apt-get update && \
	apt-get install -y ros-${ROS_DISTRO}-moveit-* && \

	rm -rf /var/lib/apt/lists/*
