cmake_minimum_required(VERSION 2.8)
project(planning_interface)

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include  
  DEPENDS planning_scene
  )
