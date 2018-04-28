cmake_minimum_required(VERSION 2.8.3)
project(moveit_experimental)

add_definitions(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED system filesystem date_time thread iostreams)
<<<<<<< HEAD
find_package(catkin REQUIRED
=======
find_package(catkin REQUIRED 
>>>>>>> upstream/indigo-devel
COMPONENTS
  moveit_core
  moveit_msgs
  resource_retriever
  kdl_parser
  geometric_shapes
<<<<<<< HEAD
  eigen_stl_containers
=======
  shape_tools
  eigen_stl_containers 
>>>>>>> upstream/indigo-devel
  eigen_conversions
  random_numbers
  visualization_msgs
  roslib
  rostime
  pluginlib
  cmake_modules
)
find_package(console_bridge REQUIRED)
<<<<<<< HEAD
find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

=======
find_package(Eigen REQUIRED)
>>>>>>> upstream/indigo-devel
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
<<<<<<< HEAD
    ${EIGEN3_INCLUDE_DIR}
=======
>>>>>>> upstream/indigo-devel
  LIBRARIES
    moveit_collision_distance_field
    collision_detector_hybrid_plugin
    ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS
    moveit_core
    geometric_shapes
<<<<<<< HEAD
=======
    shape_tools
>>>>>>> upstream/indigo-devel
    eigen_stl_containers
    eigen_conversions
    moveit_msgs
    kdl_parser
    pluginlib
  DEPENDS
<<<<<<< HEAD
    EIGEN3
    Boost
    urdfdom
    urdfdom_headers
    console_bridge
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS}
                           )

include_directories(${THIS_PACKAGE_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${console_bridge_INCLUDE_DIRS}
                    ${urdfdom_INCLUDE_DIRS}
                    ${urdfdom_headers_INCLUDE_DIRS}
=======
    Boost
    Eigen
    urdfdom
    urdfdom_headers
    console_bridge
    )

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS}
                           )

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} 
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS} 
                    ${console_bridge_INCLUDE_DIRS} 
                    ${urdfdom_INCLUDE_DIRS} 
                    ${urdfdom_headers_INCLUDE_DIRS} 
>>>>>>> upstream/indigo-devel
                    ${OCTOMAP_INCLUDE_DIRS}
                    )

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
add_subdirectory(collision_distance_field)

install(FILES collision_detector_hybrid_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Generate and install version.h
