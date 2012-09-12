cmake_minimum_required(VERSION 2.8)
project(distance_field)

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include  
  LIBRARIES ${PROJECT_NAME}
  DEPENDS moveit_msgs visualization_msgs
  )

include_directories(include)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

find_package(catkin COMPONENTS geometric_shapes moveit_msgs visualization_msgs eigen_stl_containers REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(distance_field
  src/distance_field.cpp
  src/propagation_distance_field.cpp
  )
target_link_libraries(distance_field ${catkin_LIBRARIES})

catkin_add_gtest(test_voxel_grid test/test_voxel_grid.cpp)
target_link_libraries(test_voxel_grid distance_field)

catkin_add_gtest(test_distance_field test/test_distance_field.cpp)
target_link_libraries(test_distance_field distance_field)
