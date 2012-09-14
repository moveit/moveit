cmake_minimum_required(VERSION 2.8)
project(planning_interface)

find_package(planning_scene REQUIRED)

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include  
  DEPENDS planning_scene
  )

install(DIRECTORY include/
  DESTINATION include)
