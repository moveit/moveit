set(MOVEIT_LIB_NAME moveit_ompl_interface_ros)

add_library(${MOVEIT_LIB_NAME} src/ompl_interface_ros.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_ompl_planner src/ompl_planner.cpp)
target_link_libraries(moveit_ompl_planner ${MOVEIT_LIB_NAME})

add_library(moveit_ompl_planner_plugin src/ompl_plugin.cpp)
target_link_libraries(moveit_ompl_planner_plugin ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} moveit_ompl_planner moveit_ompl_planner_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
