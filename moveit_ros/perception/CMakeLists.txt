cmake_minimum_required(VERSION 2.8.3)
project(moveit_ros_perception)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

find_package(Boost REQUIRED thread signals)
find_package(PCL REQUIRED QUIET COMPONENTS common)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
if (OPENGL_FOUND)
  set(gl_LIBS ${gl_LIBS} ${OPENGL_LIBRARIES})
endif(OPENGL_FOUND)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  roscpp
  rosconsole
  urdf
  message_filters
  tf
  tf_conversions
  pcl_conversions
  pluginlib
  image_transport
  object_recognition_msgs
  cmake_modules
)
find_package(Eigen REQUIRED)

find_package(OpenMP)
find_package(octomap REQUIRED)
find_package(OpenCV)

catkin_package(
  INCLUDE_DIRS
    mesh_filter/include
    depth_image_octomap_updater/include
    point_containment_filter/include
    occupancy_map_monitor/include
    pointcloud_octomap_updater/include
    semantic_world/include
    ${OCTOMAP_INCLUDE_DIRS}
    ${PCL_COMMON_INCLUDE_DIRS}
  LIBRARIES
    moveit_point_containment_filter
    moveit_occupancy_map_monitor
    moveit_pointcloud_octomap_updater_core
    moveit_semantic_world
    ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS
    moveit_core
    image_transport)
  
include_directories(mesh_filter/include
                    depth_image_octomap_updater/include
		    point_containment_filter/include
                    occupancy_map_monitor/include
		    pointcloud_octomap_updater/include
		    semantic_world/include
                    ${catkin_INCLUDE_DIRS}
                    )
include_directories(SYSTEM
		    ${PCL_COMMON_INCLUDE_DIRS}
		    ${OpenCV_INCLUDE_DIRS}
                    ${EIGEN_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
		    ${GLEW_INCLUDE_DIR}
		    ${GLUT_INCLUDE_DIR}
		    )

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(point_containment_filter)
add_subdirectory(occupancy_map_monitor)
add_subdirectory(pointcloud_octomap_updater)
add_subdirectory(mesh_filter)
add_subdirectory(depth_image_octomap_updater)

add_subdirectory(semantic_world)

install(
  FILES
    pointcloud_octomap_updater_plugin_description.xml
    depth_image_octomap_updater_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})
