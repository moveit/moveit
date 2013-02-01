set(MOVEIT_LIB_NAME moveit_kinematics_metrics)

add_library(${MOVEIT_LIB_NAME}
  src/kinematics_metrics.cpp)
# This line is needed to ensure that messages are done being built before this is built
add_dependencies(${MOVEIT_LIB_NAME} moveit_msgs_gencpp)

target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_state ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)
