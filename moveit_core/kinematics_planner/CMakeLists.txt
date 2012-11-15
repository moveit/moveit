set(MOVEIT_LIB_NAME moveit_kinematics_planner)

add_library(${MOVEIT_LIB_NAME}
  src/kinematics_solver.cpp
  src/kinematics_planner.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
