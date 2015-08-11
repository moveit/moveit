cmake_minimum_required(VERSION 2.8.3)
project(moveit_experimental)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED system filesystem date_time thread iostreams)
find_package(catkin REQUIRED 
COMPONENTS
  moveit_core
  moveit_msgs
  resource_retriever
  srdfdom
  kdl_parser
  geometric_shapes
  shape_tools
  eigen_stl_containers 
  eigen_conversions
  random_numbers
  visualization_msgs
  roslib
  rostime
  cmake_modules
)
find_package(console_bridge REQUIRED)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED fcl)
find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

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
  LIBRARIES
    moveit_collision_distance_field
    ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS
    geometric_shapes
    shape_tools
    eigen_stl_containers
    eigen_conversions
    random_numbers
    moveit_msgs
    kdl_parser
    srdfdom
  DEPENDS
    Boost
    Eigen
    urdfdom
    urdfdom_headers
    console_bridge
    )

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS}
                           ${Boost_INCLUDE_DIR}
                           )

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} 
                    ${catkin_INCLUDE_DIRS} 
                    ${console_bridge_INCLUDE_DIRS} 
                    ${urdfdom_INCLUDE_DIRS} 
                    ${urdfdom_headers_INCLUDE_DIRS} 
                    ${OCTOMAP_INCLUDE_DIRS}
                    )

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
add_subdirectory(collision_distance_field)

# Generate and install version.h
