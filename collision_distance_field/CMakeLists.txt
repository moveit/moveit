set(MOVEIT_LIB_NAME moveit_collision_distance_field)

add_library(${MOVEIT_LIB_NAME}
  src/collision_distance_field_types.cpp
  src/collision_common_distance_field.cpp
  src/collision_robot_distance_field.cpp
  src/collision_world_distance_field.cpp
  src/hybrid_collision_robot.cpp
  src/hybrid_collision_world.cpp
)
target_link_libraries(${MOVEIT_LIB_NAME} moveit_collision_detection_fcl moveit_distance_field ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

catkin_add_gtest(test_collision_distance_field test/test_collision_distance_field.cpp)
target_link_libraries(test_collision_distance_field  ${MOVEIT_LIB_NAME})
