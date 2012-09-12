cmake_minimum_required(VERSION 2.8)
project(trajectory_processing)

include_directories(include)

find_package(planning_scene REQUIRED)
include_directories(${planning_scene_INCLUDE_DIRS})
link_directories(${planning_scene_LIBRARY_DIRS})

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include  
  LIBRARIES ${PROJECT_NAME}
  DEPENDS planning_scene
  )

add_library(${PROJECT_NAME}
  src/iterative_smoother.cpp
  src/trajectory_tools.cpp
)
target_link_libraries(${PROJECT_NAME} ${planning_scene_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)
