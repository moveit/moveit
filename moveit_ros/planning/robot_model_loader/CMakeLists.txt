add_library(moveit_robot_model_loader src/robot_model_loader.cpp)
target_link_libraries(moveit_robot_model_loader ${catkin_LIBRARIES})

install(TARGETS moveit_robot_model_loader LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
