set(CMAKE_BUILD_TYPE Debug)
set(MOVEIT_LIB_NAME moveit_semantic_world)

add_library(${MOVEIT_LIB_NAME} src/semantic_world.cpp)
add_dependencies(${MOVEIT_LIB_NAME} moveit_msgs_generate_messages_cpp )
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
