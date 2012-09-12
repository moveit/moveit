cmake_minimum_required(VERSION 2.8)
project(planning_request_adapter)

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include  
  LIBRARIES ${PROJECT_NAME}
  DEPENDS planning_interface
  )

include_directories(include)

find_package(Boost REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(planning_interface REQUIRED)
include_directories(${planning_interface_INCLUDE_DIRS})
link_directories(${planning_interface_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/planning_request_adapter.cpp)
target_link_libraries(${PROJECT_NAME} ${planning_interface_LIBRARIES} ${Boost_LIBRARIES})
