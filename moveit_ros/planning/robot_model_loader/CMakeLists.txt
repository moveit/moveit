set(MOVEIT_LIB_NAME moveit_robot_model_loader)

add_library(${MOVEIT_LIB_NAME} src/robot_model_loader.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION 0.7.3)
target_link_libraries(${MOVEIT_LIB_NAME} moveit_rdf_loader moveit_kinematics_plugin_loader ${catkin_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
