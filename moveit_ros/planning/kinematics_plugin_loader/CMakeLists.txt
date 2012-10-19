add_library(moveit_kinematics_plugin_loader src/kinematics_plugin_loader.cpp)
target_link_libraries(moveit_kinematics_plugin_loader ${catkin_LIBRARIES})

install(TARGETS moveit_kinematics_plugin_loader LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
