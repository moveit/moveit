# moveit/moveit:noetic-source
# Downloads the moveit source code and install remaining debian dependencies

FROM moveit/moveit:noetic-ci-shadow-fixed
MAINTAINER Dave Coleman dave@picknik.ai

ENV ROS_UNDERLAY /root/ws_moveit/install
WORKDIR $ROS_UNDERLAY/../src

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
RUN \
    # Download moveit source so that we can get necessary dependencies
    wstool init . https://raw.githubusercontent.com/ros-planning/moveit/${ROS_DISTRO}-devel/moveit.rosinstall && \
    #
    # Update apt package list as cache is cleared in previous container
    # Usually upgrading involves a few packages only (if container builds became out-of-sync)
    apt-get -qq update && \
    apt-get -qq dist-upgrade && \
    #
    rosdep update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    rm -rf /var/lib/apt/lists/*

ENV PYTHONIOENCODING UTF-8
RUN cd .. && \
    catkin config --extend /opt/ros/$ROS_DISTRO --install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \
    # Status rate is limited so that just enough info is shown to keep Docker from timing out,
    # but not too much such that the Docker log gets too long (another form of timeout)
    catkin build --limit-status-rate 0.001 --no-notify

# Environment variable used in instructions on moveit.ros.org website for running clang-tidy
ENV CATKIN_WS /root/ws_moveit
