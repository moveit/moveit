cmake_minimum_required(VERSION 2.8.3)
project(moveit_ros_perception)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread signals)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  roscpp
  rosconsole
  urdf
  message_filters
  tf
  tf_conversions
  pcl_ros
)

catkin_package(
  LIBRARIES  
    moveit_robot_self_filter
    moveit_occupancy_map_monitor
  INCLUDE_DIRS
    robot_self_filter/include
    occupancy_map_monitor/include
  CATKIN_DEPENDS
    moveit_core)

include_directories(robot_self_filter/include
                    occupancy_map_monitor/include
                    ${catkin_INCLUDE_DIRS} 
                    ${EIGEN_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    )

add_subdirectory(robot_self_filter)
add_subdirectory(occupancy_map_monitor)
