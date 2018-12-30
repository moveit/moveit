cmake_minimum_required(VERSION 2.8.3)
project(moveit_experimental)

add_definitions(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED system filesystem date_time thread iostreams)
find_package(catkin REQUIRED
COMPONENTS
  eigen_conversions
  eigen_stl_containers
  geometric_shapes
  geometry_msgs
  kdl_parser
  moveit_core
  moveit_msgs
  pluginlib
  rosconsole
  roslib
  rostime
  std_msgs
  trajectory_msgs
  visualization_msgs
  # The following are needed by jog_arm:
  moveit_ros_planning_interface
  rosparam_shortcuts
  tf2_ros
)
find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(octomap REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)

set(VERSION_FILE_PATH "${CATKIN_DEVEL_PREFIX}/include")
file(MAKE_DIRECTORY "${VERSION_FILE_PATH}")

set(THIS_PACKAGE_INCLUDE_DIRS
  ${VERSION_FILE_PATH}
  collision_distance_field/include
  jog_arm/include
)

add_message_files(
  FILES
  JogJoint.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    moveit_collision_distance_field
    collision_detector_hybrid_plugin
    ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS
    eigen_conversions
    eigen_stl_containers
    geometry_msgs
    geometric_shapes
    kdl_parser
    message_runtime
    moveit_core
    moveit_msgs
    pluginlib
    std_msgs
    trajectory_msgs
    visualization_msgs
    # The following are needed by jog_arm:
    joy_teleop
    moveit_ros_planning_interface
    rosparam_shortcuts
    tf2_ros
  DEPENDS
    EIGEN3
    Boost
    urdfdom
    urdfdom_headers
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS}
                           )

include_directories(${THIS_PACKAGE_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${urdfdom_INCLUDE_DIRS}
                    ${urdfdom_headers_INCLUDE_DIRS}
                    ${OCTOMAP_INCLUDE_DIRS}
                    )

add_subdirectory(collision_distance_field)
add_subdirectory(jog_arm)

install(FILES collision_detector_hybrid_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Generate and install version.h
