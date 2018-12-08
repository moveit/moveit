cmake_minimum_required(VERSION 2.8.3)
project(moveit_collision_distance_field)

add_compile_options(-std=c++14)

find_package(Boost REQUIRED thread filesystem)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  visualization_msgs
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS moveit_core visualization_msgs
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/collision_distance_field_types.cpp
  src/collision_common_distance_field.cpp
  src/collision_robot_distance_field.cpp
  src/collision_world_distance_field.cpp
  src/collision_robot_hybrid.cpp
  src/collision_world_hybrid.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

set(MOVEIT_LIB_NAME moveit_collision_detector_hybrid_plugin)
add_library(${MOVEIT_LIB_NAME} src/collision_detector_hybrid_plugin_loader.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME})

if(CATKIN_ENABLE_TESTING)
  find_package(moveit_resources REQUIRED)
  include_directories(${moveit_resources_INCLUDE_DIRS})

  catkin_add_gtest(test_collision_distance_field test/test_collision_distance_field.cpp)
  target_link_libraries(test_collision_distance_field  ${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME} ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(FILES collision_detector_hybrid_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
