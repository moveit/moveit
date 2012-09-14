cmake_minimum_required(VERSION 2.8)
project(planning_scene)

include_directories(include)

find_package(Boost REQUIRED thread date_time)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(OCTOMAP REQUIRED octomap)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})

find_package(collision_detection_fcl REQUIRED)
include_directories(${collision_detection_fcl_INCLUDE_DIRS})
link_directories(${collision_detection_fcl_LIBRARY_DIRS})

find_package(kinematic_constraints REQUIRED)
include_directories(${kinematic_constraints_INCLUDE_DIRS})
link_directories(${kinematic_constraints_LIBRARY_DIRS})

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include  
  LIBRARIES ${PROJECT_NAME}
  DEPENDS collision_detection kinematic_constraints 
  )

add_library(${PROJECT_NAME} src/planning_scene.cpp)
target_link_libraries(${PROJECT_NAME} ${collision_detection_fcl_LIBRARIES}
  ${kinematic_constraints_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_planning_scene test/test_planning_scene.cpp)
target_link_libraries(test_planning_scene ${PROJECT_NAME})
