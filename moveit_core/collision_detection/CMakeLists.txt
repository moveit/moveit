set(MOVEIT_LIB_NAME moveit_collision_detection)

add_library(${MOVEIT_LIB_NAME}
  src/collision_world.cpp 
  src/collision_robot.cpp
  src/collision_matrix.cpp
  src/collision_tools.cpp
  src/collision_octomap_filter.cpp
  src/allvalid/collision_robot.cpp
  src/allvalid/collision_world.cpp
)

target_link_libraries(${MOVEIT_LIB_NAME} moveit_kinematic_state ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

