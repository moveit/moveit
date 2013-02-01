set(MOVEIT_LIB_NAME moveit_constraint_samplers)

add_library(${MOVEIT_LIB_NAME}
  src/constraint_sampler.cpp
  src/default_constraint_samplers.cpp
  src/union_constraint_sampler.cpp
  src/constraint_sampler_manager.cpp
  src/constraint_sampler_tools.cpp
)
# This line is needed to ensure that messages are done being built before this is built
add_dependencies(${MOVEIT_LIB_NAME} moveit_msgs_gencpp)
target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_state 
  moveit_kinematic_constraints
  moveit_planning_scene 
  ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

catkin_add_gtest(test_constraint_samplers test/test_constraint_samplers.cpp test/pr2_arm_kinematics_plugin.cpp test/pr2_arm_ik.cpp)
target_link_libraries(test_constraint_samplers ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES})
