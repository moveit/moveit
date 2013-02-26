cmake_minimum_required(VERSION 2.8.3)
project(moveit_ros_perception)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

include(${PROJECT_SOURCE_DIR}/cmake/moveit_find_sse.cmake)
moveit_check_for_sse()

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
if (OPENGL_FOUND)
  set(gl_LIBS ${gl_LIBS} ${OPENGL_LIBRARIES})
endif(OPENGL_FOUND)

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
  image_transport
)

find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS
    mesh_filter/include
    robot_self_filter/include
    occupancy_map_monitor/include
    ${OCTOMAP_INCLUDE_DIRS}
  LIBRARIES
    moveit_mesh_filter
    moveit_robot_self_filter
    moveit_occupancy_map_monitor
    ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS
    moveit_core)

include_directories(mesh_filter/include
                    robot_self_filter/include
                    occupancy_map_monitor/include
                    ${catkin_INCLUDE_DIRS} 
                    )
include_directories(SYSTEM
                    ${EIGEN_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
		    ${GLEW_INCLUDE_DIR}
		    ${GLUT_INCLUDE_DIR}
		    )

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(mesh_filter)
add_subdirectory(robot_self_filter)
add_subdirectory(occupancy_map_monitor)
