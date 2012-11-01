set(MOVEIT_LIB_NAME moveit_kinematics_metrics)

add_library(${MOVEIT_LIB_NAME}
  src/kinematics_metrics.cpp)

target_link_libraries(${MOVEIT_LIB_NAME} moveit_kinematic_state ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)
