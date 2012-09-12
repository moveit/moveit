cmake_minimum_required(VERSION 2.8)
project(sbpl_interface)

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include  
  LIBRARIES ${PROJECT_NAME}
  DEPENDS planning_scene collision_distance_field
  )

include_directories(include)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

find_package(planning_scene REQUIRED)
include_directories(${planning_scene_INCLUDE_DIRS})
link_directories(${planning_scene_LIBRARY_DIRS})

find_package(collision_distance_field REQUIRED)
include_directories(${collision_distance_field_INCLUDE_DIRS})
link_directories(${collision_distance_field_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/bfs3d/BFS_3D.cpp
  src/bfs3d/Search.cpp
  src/sbpl_interface.cpp
  src/sbpl_meta_interface.cpp
  src/environment_chain3d.cpp
)
target_link_libraries(${PROJECT_NAME} ${planning_scene_LIBRARIES} ${collision_distance_field_LIBRARIES} ${SBPL_LIBRARIES})

