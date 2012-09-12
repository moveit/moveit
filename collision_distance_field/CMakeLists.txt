cmake_minimum_required(VERSION 2.8)
project(collision_distance_field)

include_directories(include)

find_package(collision_detection_fcl REQUIRED)
include_directories(${collision_detection_fcl_INCLUDE_DIRS})
link_directories(${collision_detection_fcl_LIBRARY_DIRS})

find_package(distance_field REQUIRED)
include_directories(${distance_field_INCLUDE_DIRS})
link_directories(${distance_field_LIBRARY_DIRS})

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include  
  LIBRARIES ${PROJECT_NAME}
  DEPENDS collision_detection_fcl distance_field
  )

add_library(${PROJECT_NAME}
  src/collision_distance_field_types.cpp
  src/collision_common_distance_field.cpp
  src/collision_robot_distance_field.cpp
  src/collision_world_distance_field.cpp
  src/hybrid_collision_robot.cpp
  src/hybrid_collision_world.cpp
)
target_link_libraries(${PROJECT_NAME} ${distance_field_LIBRARIES} ${collision_detection_fcl_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)

catkin_add_gtest(test_collision_distance_field test/test_collision_distance_field.cpp)
target_link_libraries(test_collision_distance_field ${PROJECT_NAME})
