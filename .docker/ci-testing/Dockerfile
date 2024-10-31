# moveit/moveit:noetic-ci-testing
# Sets up a base image to use for running Continuous Integration on Travis

ARG IMAGE=noetic
FROM moveit/moveit:${IMAGE}-ci
LABEL org.opencontainers.image.authors="Robert Haschke <rhaschke@techfak.uni-bielefeld.de>"

# Switch to ros-testing
RUN apt-get update && \
    apt-get install curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros-testing/ubuntu `lsb_release -cs` main" | tee /etc/apt/sources.list.d/ros1-latest.list && \
    # Upgrade packages to ros-testing and clean apt-cache within one RUN command
    apt-get update && \
    apt-get -qq -y dist-upgrade && \
    #
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*
