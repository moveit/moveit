set(MOVEIT_LIB_NAME moveit_planning_models_loader)

add_library(${MOVEIT_LIB_NAME} src/kinematic_model_loader.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_model_loader moveit_kinematics_plugin_loader ${catkin_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
