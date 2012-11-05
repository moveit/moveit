
#add_executable(moveit_environment_server src/environment_server.cpp) # we do not install this one; it should not be used
#target_link_libraries(moveit_environment_server moveit_planning_scene_monitor ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_print_planning_model_info src/print_planning_model_info.cpp)
target_link_libraries(moveit_print_planning_model_info moveit_planning_scene_monitor ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_display_random_state src/display_random_state.cpp)
target_link_libraries(moveit_display_random_state moveit_planning_scene_monitor ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_visualize_robot_collision_volume src/visualize_robot_collision_volume.cpp)
target_link_libraries(moveit_visualize_robot_collision_volume moveit_planning_scene_monitor ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_evaluate_collision_checking_speed src/evaluate_collision_checking_speed.cpp)
target_link_libraries(moveit_evaluate_collision_checking_speed moveit_planning_scene_monitor ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# The constraints aware IK will be rewritten before release;
# add_executable(moveit_inverse_kinematics_sanity_checker src/inverse_kinematics_sanity_checker_main.cpp)

install(TARGETS
  moveit_print_planning_model_info 
  moveit_display_random_state
  moveit_visualize_robot_collision_volume
  moveit_evaluate_collision_checking_speed
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# this test should not be in this package
#catkin_add_gtest(test_planning_scene_distance_field src/test_planning_scene_distance_field.cpp)
