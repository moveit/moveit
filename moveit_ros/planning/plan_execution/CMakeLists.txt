set(MOVEIT_LIB_NAME moveit_plan_execution)

add_library(${MOVEIT_LIB_NAME}
  src/plan_with_sensing.cpp
  src/plan_execution.cpp)
<<<<<<< HEAD
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
=======
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION 0.7.3)
>>>>>>> upstream/indigo-devel
target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_planning_pipeline
  moveit_trajectory_execution_manager
  moveit_planning_scene_monitor
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
  )

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
