set(MOVEIT_LIB_NAME moveit_robot_interaction)

add_library(${MOVEIT_LIB_NAME}
  src/interactive_marker_helpers.cpp
  src/robot_interaction.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
