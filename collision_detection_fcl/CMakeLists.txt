cmake_minimum_required(VERSION 2.8)
project(collision_detection_fcl)

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include  
  LIBRARIES ${PROJECT_NAME}
  DEPENDS collision_detection
  )

include_directories(include)

find_package(Boost REQUIRED thread system filesystem)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(FCL REQUIRED fcl)
include_directories(${FCL_INCLUDE_DIRS})
link_directories(${FCL_LIBRARY_DIRS})

find_package(collision_detection REQUIRED)
include_directories(${collision_detection_INCLUDE_DIRS})
link_directories(${collision_detection_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/collision_common.cpp
  src/collision_robot.cpp
  src/collision_world.cpp
)

target_link_libraries(${PROJECT_NAME} ${collision_detection_LIBRARIES} ${FCL_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_fcl_collision_detection test/test_fcl_collision_detection.cpp)
target_link_libraries(test_fcl_collision_detection ${PROJECT_NAME} ${Boost_LIBRARIES})
