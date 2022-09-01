# moveit/moveit:melodic-source
# Downloads the moveit source code and install remaining debian dependencies

FROM moveit/moveit:melodic-ci-shadow-fixed
MAINTAINER Robert Haschke rhaschke@techfak.uni-bielefeld.de

ENV PYTHONIOENCODING UTF-8
# Export ROS_UNDERLAY for downstream docker containers
ENV ROS_UNDERLAY /root/ws_moveit/install
# Environment variable used in instructions on moveit.ros.org website for running clang-tidy
ENV CATKIN_WS $(realpath $ROS_UNDERLAY/..)
WORKDIR $ROS_UNDERLAY/..

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
RUN \
    # Fetch required upstream sources for building
    git clone --depth 1 --branch ${ROS_DISTRO}-devel https://github.com/ros-planning/moveit src/moveit && \
    git clone --depth 1 --branch ${ROS_DISTRO}-devel https://github.com/ros-planning/moveit_resources src/moveit_resources && \
    wstool init --shallow src src/moveit/.github/workflows/upstream.rosinstall && \
    #
    catkin config --extend /opt/ros/$ROS_DISTRO --install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \
    # Status rate is limited so that just enough info is shown to keep Docker from timing out,
    # but not too much such that the Docker log gets too long (another form of timeout)
    catkin build --limit-status-rate 0.001 --no-notify && \
    #
    # Update /ros_entrypoint.sh to source our new workspace
    sed -i "s#/opt/ros/\$ROS_DISTRO/setup.bash#$ROS_UNDERLAY/setup.sh#g" /ros_entrypoint.sh
