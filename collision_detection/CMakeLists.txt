set(MOVEIT_LIB_NAME moveit_collision_detection)

add_library(${MOVEIT_LIB_NAME}
  src/world.cpp 
  src/world_diff.cpp 
  src/collision_world.cpp 
  src/collision_robot.cpp
  src/collision_matrix.cpp
  src/collision_tools.cpp
  src/collision_octomap_filter.cpp
  src/allvalid/collision_robot_allvalid.cpp
  src/allvalid/collision_world_allvalid.cpp
)
# This line is needed to ensure that messages are done being built before this is built
add_dependencies(${MOVEIT_LIB_NAME} moveit_msgs_gencpp)

target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_state ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# unit tests
catkin_add_gtest(test_world test/test_world.cpp)
target_link_libraries(test_world ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_world_diff test/test_world_diff.cpp)
target_link_libraries(test_world_diff ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

