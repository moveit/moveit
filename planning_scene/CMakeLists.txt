set(MOVEIT_LIB_NAME moveit_planning_scene)

add_library(${MOVEIT_LIB_NAME} src/planning_scene.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} 
  moveit_collision_detection_fcl moveit_kinematic_constraints
  ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

catkin_add_gtest(test_planning_scene test/test_planning_scene.cpp)
target_link_libraries(test_planning_scene ${MOVEIT_LIB_NAME})
