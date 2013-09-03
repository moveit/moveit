set(MOVEIT_LIB_NAME moveit_planning_request_adapter)

add_library(${MOVEIT_LIB_NAME} src/planning_request_adapter.cpp)

target_link_libraries(${MOVEIT_LIB_NAME} moveit_planning_scene ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)
