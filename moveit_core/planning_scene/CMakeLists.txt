set(MOVEIT_LIB_NAME moveit_planning_scene)

add_library(${MOVEIT_LIB_NAME} src/planning_scene.cpp)
# This line is needed to ensure that messages are done being built before this is built
add_dependencies(${MOVEIT_LIB_NAME} moveit_msgs_gencpp)
target_link_libraries(${MOVEIT_LIB_NAME} 
  moveit_robot_model
  moveit_robot_state
  moveit_collision_detection_fcl
  moveit_kinematic_constraints 
  moveit_robot_trajectory
  moveit_trajectory_processing
  ${LIBOCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

catkin_add_gtest(test_planning_scene test/test_planning_scene.cpp)
target_link_libraries(test_planning_scene ${MOVEIT_LIB_NAME})
