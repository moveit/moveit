set(MOVEIT_LIB_NAME moveit_trajectory_execution_manager)

add_library(${MOVEIT_LIB_NAME} src/trajectory_execution_manager.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${MOVEIT_LIB_NAME} moveit_planning_scene_monitor moveit_robot_model_loader ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${moveit_ros_planning_EXPORTED_TARGETS}) # don't build until necessary msgs are finish

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

add_library(test_controller_manager_plugin test/test_moveit_controller_manager_plugin.cpp)
set_target_properties(test_controller_manager_plugin PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(test_controller_manager_plugin ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_controller_manager test/test_app.cpp)
target_link_libraries(test_controller_manager ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
