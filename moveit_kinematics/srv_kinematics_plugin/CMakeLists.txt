set(MOVEIT_LIB_NAME moveit_srv_kinematics_plugin)

add_library(${MOVEIT_LIB_NAME} src/srv_kinematics_plugin.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION 0.7.3)

target_link_libraries(${MOVEIT_LIB_NAME} moveit_rdf_loader ${catkin_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
