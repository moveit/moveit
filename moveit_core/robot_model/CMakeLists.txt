set(MOVEIT_LIB_NAME moveit_robot_model)

add_library(${MOVEIT_LIB_NAME}
  src/link_model.cpp
  src/joint_model.cpp
  src/fixed_joint_model.cpp
  src/revolute_joint_model.cpp
  src/prismatic_joint_model.cpp
  src/planar_joint_model.cpp
  src/floating_joint_model.cpp
  src/joint_model_group.cpp
  src/robot_model.cpp
  )

target_link_libraries(${MOVEIT_LIB_NAME} moveit_profiler moveit_exceptions moveit_kinematics_base ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

if(${CATKIN_ENABLE_TESTING})
  catkin_add_gtest(test_robot_model test/test.cpp)
  target_link_libraries(test_robot_model ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME})
endif()

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
