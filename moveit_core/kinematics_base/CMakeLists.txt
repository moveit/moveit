cmake_minimum_required(VERSION 2.8)
project(kinematics_base)

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include)

install(DIRECTORY include/
  DESTINATION include)
