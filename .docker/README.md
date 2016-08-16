# MoveIt! Docker Images
This repo hosts the Dockerfiles used to generate images for [MoveIt!](moveit.ros.org) :whale:

[![Docker Pulls](https://img.shields.io/docker/pulls/moveit/moveit.svg?maxAge=2592000)](https://hub.docker.com/r/moveit/moveit/)
[![Docker Stars](https://img.shields.io/docker/stars/moveit/moveit.svg)](https://registry.hub.docker.com/moveit/moveit/)

## Available Images

For each ROS distribution there are 3 images, built on top of a standard [osrf/ros:jade-desktop](https://github.com/osrf/docker_images/blob/master/ros/jade/jade-desktop/Dockerfile) (or other distro version) image:

 - **source image**: contains all dependencies and a full MoveIt! workspace downloaded and built to ~/ws_moveit/src
 - **release image**: the full debian-based install of MoveIt! using apt-get
 - **ci image**: an image optimized for running continuous integration with Travis and [moveit_ci](https://github.com/ros-planning/moveit_ci)
