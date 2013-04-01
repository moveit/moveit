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

target_link_libraries(${MOVEIT_LIB_NAME} moveit_profiler ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# This line is needed to ensure that messages are done being built before this is built
add_dependencies(${MOVEIT_LIB_NAME} moveit_msgs_gencpp)

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
