set(MOVEIT_LIB_NAME moveit_robot_state)

add_library(${MOVEIT_LIB_NAME}
  src/link_state.cpp
  src/joint_state.cpp
  src/joint_state_group.cpp
  src/attached_body.cpp
  src/robot_state.cpp
  src/transforms.cpp
  src/conversions.cpp
)
# This line is needed to ensure that messages are done being built before this is built
add_dependencies(${MOVEIT_LIB_NAME} moveit_msgs_gencpp)

target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_model ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# Unit tests
catkin_add_gtest(test_robot_state test/test_kinematic.cpp)
target_link_libraries(test_robot_state ${MOVEIT_LIB_NAME})

catkin_add_gtest(test_robot_state_complex test/test_kinematic_complex.cpp)
target_link_libraries(test_robot_state_complex ${MOVEIT_LIB_NAME})

catkin_add_gtest(test_transforms test/test_transforms.cpp)
target_link_libraries(test_transforms ${MOVEIT_LIB_NAME})
