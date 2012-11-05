set(MOVEIT_LIB_NAME moveit_dynamics_solver)

add_library(${MOVEIT_LIB_NAME}
  src/dynamics_solver.cpp)

target_link_libraries(${MOVEIT_LIB_NAME} moveit_kinematic_state ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)
