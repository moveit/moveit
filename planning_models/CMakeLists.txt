set(MOVEIT_LIB_NAME moveit_planning_models)

add_library(${MOVEIT_LIB_NAME}
  src/kinematic_model.cpp
  src/kinematic_model/joint_model.cpp
  src/kinematic_model/fixed_joint_model.cpp
  src/kinematic_model/planar_joint_model.cpp
  src/kinematic_model/floating_joint_model.cpp
  src/kinematic_model/prismatic_joint_model.cpp
  src/kinematic_model/revolute_joint_model.cpp
  src/kinematic_model/link_model.cpp
  src/kinematic_model/joint_model_group.cpp
  src/kinematic_state.cpp
  src/kinematic_state/joint_state.cpp
  src/kinematic_state/link_state.cpp
  src/kinematic_state/attached_body.cpp
  src/kinematic_state/joint_state_group.cpp
  src/transforms.cpp
  src/conversions.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# Unit tests
catkin_add_gtest(test_kinematic test/test_kinematic.cpp)
target_link_libraries(test_kinematic ${MOVEIT_LIB_NAME})

catkin_add_gtest(test_kinematic_complex test/test_kinematic_complex.cpp)
target_link_libraries(test_kinematic_complex ${MOVEIT_LIB_NAME})

catkin_add_gtest(test_transforms test/test_transforms.cpp)
target_link_libraries(test_transforms ${MOVEIT_LIB_NAME})
