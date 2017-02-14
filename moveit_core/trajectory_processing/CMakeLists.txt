set(MOVEIT_LIB_NAME moveit_trajectory_processing)

add_library(${MOVEIT_LIB_NAME}
  src/iterative_time_parameterization.cpp
  src/trajectory_tools.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION 0.7.3)

target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_state moveit_robot_trajectory ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
