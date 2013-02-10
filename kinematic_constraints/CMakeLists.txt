set(MOVEIT_LIB_NAME moveit_kinematic_constraints)

add_library(${MOVEIT_LIB_NAME}
  src/kinematic_constraint.cpp
  src/utils.cpp)
# This line is needed to ensure that messages are done being built before this is built
add_dependencies(${MOVEIT_LIB_NAME} moveit_msgs_gencpp)

target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_model moveit_robot_state moveit_collision_detection_fcl
  ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

catkin_add_gtest(test_constraints test/test_constraints.cpp)
target_link_libraries(test_constraints  ${MOVEIT_LIB_NAME})
