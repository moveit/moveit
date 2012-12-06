add_executable(test_kinematics_planner src/test_kinematics_planner.cpp)
target_link_libraries(test_kinematics_planner 
  ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS test_kinematics_planner RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
