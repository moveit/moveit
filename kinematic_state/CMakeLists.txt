set(MOVEIT_LIB_NAME moveit_kinematic_state)

add_library(${MOVEIT_LIB_NAME}
  src/link_state.cpp
  src/joint_state.cpp
  src/joint_state_group.cpp
  src/attached_body.cpp
  src/kinematic_state.cpp
  src/transforms.cpp
  src/conversions.cpp
)

target_link_libraries(${MOVEIT_LIB_NAME} moveit_kinematic_model ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# Unit tests
catkin_add_gtest(test_kinematic test/test_kinematic.cpp)
target_link_libraries(test_kinematic ${MOVEIT_LIB_NAME})

catkin_add_gtest(test_kinematic_complex test/test_kinematic_complex.cpp)
target_link_libraries(test_kinematic_complex ${MOVEIT_LIB_NAME})

catkin_add_gtest(test_transforms test/test_transforms.cpp)
target_link_libraries(test_transforms ${MOVEIT_LIB_NAME})
