cmake_minimum_required(VERSION 2.8.3)
project (mesh_filter)

if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )
include(${PROJECT_SOURCE_DIR}/cmake/pcl_find_sse.cmake)
PCL_CHECK_FOR_SSE ()

FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(GLEW REQUIRED)
FIND_PACKAGE(GLUT REQUIRED)
find_package(Eigen REQUIRED)
include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})
include_directories(${GLEW_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})

find_package(catkin COMPONENTS shape_msgs sensor_msgs moveit_core opencv2 image_transport tf moveit_ros_planning tf_conversions cv_bridge)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS sensor_msgs
  DEPENDS Eigen
  )

include_directories(${catkin_INCLUDE_DIRS} include)
link_directories(${catkin_LIBRARY_DIRS})

IF(OPENGL_FOUND)
  SET(gl_LIBS ${gl_LIBS} ${OPENGL_LIBRARIES})
ENDIF(OPENGL_FOUND)

add_library(${PROJECT_NAME}
	src/mesh_filter_base.cpp
	src/sensor_model.cpp
	src/stereo_camera_model.cpp
	src/gl_renderer.cpp
	src/gl_mesh.cpp
	src/transform_provider.cpp
  src/depth_self_filter_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${gl_LIBS} glut GLEW)

