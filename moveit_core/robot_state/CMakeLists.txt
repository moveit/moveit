set(MOVEIT_LIB_NAME moveit_robot_state)

add_library(${MOVEIT_LIB_NAME}
  src/robot_state.cpp
  src/attached_body.cpp
  src/conversions.cpp
)

target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_model moveit_kinematics_base moveit_transforms ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# Unit tests
catkin_add_gtest(test_robot_state test/test_kinematic.cpp)
target_link_libraries(test_robot_state ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME})

catkin_add_gtest(test_robot_state_complex test/test_kinematic_complex.cpp)
target_link_libraries(test_robot_state_complex ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME})
