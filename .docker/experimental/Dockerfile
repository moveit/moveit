# moveit/moveit:jade-experimental
# Based on a moveit source install, adds the mongo driver and the warehouse packages

FROM moveit/moveit:jade-source

ENV CATKIN_WS=/root/ws_moveit
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# installing mongocxx driver for the warehouse
RUN git clone -b 26compat https://github.com/mongodb/mongo-cxx-driver.git && \
    apt-get update -qq && \
    apt-get -qq install -y scons mongodb && \
    cd mongo-cxx-driver && \
    scons --use-system-boost --prefix=/usr/local/ --full --disable-warnings-as-errors && \
    ls /usr/local/lib && \
    # scons install && \
    rm -rf /var/lib/apt/lists/*

# Download warehouse source
RUN wstool set -yu warehouse_ros_mongo --git https://github.com/ros-planning/warehouse_ros_mongo.git -v jade-devel && \
    wstool set -yu warehouse_ros --git https://github.com/ros-planning/warehouse_ros.git -v jade-devel

# build the workspace
RUN cd $CATKIN_WS && \
    source /root/ws_moveit/devel/setup.bash && \
    catkin init  && \
    catkin build
