set(MOVEIT_LIB_NAME moveit_dynamics_solver)

add_library(${MOVEIT_LIB_NAME}
  src/dynamics_solver.cpp)

target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_state ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)
