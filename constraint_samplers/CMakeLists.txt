cmake_minimum_required(VERSION 2.8)
project(constraint_samplers)

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include  
  LIBRARIES ${PROJECT_NAME}
  DEPENDS planning_models kinematic_constraints
  )

include_directories(include)

find_package(Boost REQUIRED thread)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(planning_scene REQUIRED)
include_directories(${planning_scene_INCLUDE_DIRS})
link_directories(${planning_scene_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/constraint_sampler.cpp
  src/default_constraint_samplers.cpp
  src/union_constraint_sampler.cpp
  src/constraint_sampler_manager.cpp
  src/constraint_sampler_tools.cpp
)
target_link_libraries(${PROJECT_NAME} ${planning_scene_LIBRARIES} ${Boost_LIBRARIES})
