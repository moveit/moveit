set(MOVEIT_LIB_NAME moveit_collision_fcl_detailed)

add_library(${MOVEIT_LIB_NAME}
  src/collision_common_detailed.cpp
  src/collision_robot_fcl_detailed.cpp
  src/collision_world_fcl_detailed.cpp
)
# This line is needed to ensure that messages are done being built before this is built
add_dependencies(${MOVEIT_LIB_NAME} moveit_msgs_gencpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)

