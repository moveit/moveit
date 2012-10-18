add_library(collision_distance_field
  src/collision_distance_field_types.cpp
  src/collision_common_distance_field.cpp
  src/collision_robot_distance_field.cpp
  src/collision_world_distance_field.cpp
  src/hybrid_collision_robot.cpp
  src/hybrid_collision_world.cpp
)
target_link_libraries(collision_distance_field collision_detection_fcl distance_field ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS collision_distance_field
  LIBRARY DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)

catkin_add_gtest(test_collision_distance_field test/test_collision_distance_field.cpp)
target_link_libraries(test_collision_distance_field collision_distance_field)
