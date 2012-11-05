set(MOVEIT_LIB_NAME moveit_plan_execution)

add_library(${MOVEIT_LIB_NAME} src/plan_execution.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} 
  moveit_planning_pipeline
  moveit_trajectory_execution_manager
  moveit_planning_scene_monitor
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
  )

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
