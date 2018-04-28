set(MOVEIT_LIB_NAME moveit_kinematics_plugin_loader)

add_library(${MOVEIT_LIB_NAME} src/kinematics_plugin_loader.cpp)
<<<<<<< HEAD
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
=======
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION 0.7.3)
>>>>>>> upstream/indigo-devel
target_link_libraries(${MOVEIT_LIB_NAME} moveit_rdf_loader ${catkin_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
