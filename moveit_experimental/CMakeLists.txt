cmake_minimum_required(VERSION 2.8.3)
project(moveit_experimental)

add_definitions(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED system filesystem date_time thread iostreams)
find_package(catkin REQUIRED
COMPONENTS
  moveit_core
  moveit_msgs
  resource_retriever
  kdl_parser
  geometric_shapes
  eigen_stl_containers
  tf2_eigen
  random_numbers
  visualization_msgs
  roslib
  rostime
  rosconsole
  pluginlib
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
    moveit_core
    geometric_shapes
    eigen_stl_containers
    moveit_msgs
    kdl_parser
    pluginlib
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

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
add_subdirectory(collision_distance_field)

install(FILES collision_detector_hybrid_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Generate and install version.h
