set(MOVEIT_LIB_NAME moveit_collision_detection_fcl)

add_library(${MOVEIT_LIB_NAME}
  src/collision_common.cpp
  src/collision_robot_fcl.cpp
  src/collision_world_fcl.cpp
)
# This line is needed to ensure that messages are done being built before this is built
add_dependencies(${MOVEIT_LIB_NAME} moveit_msgs_gencpp)

target_link_libraries(${MOVEIT_LIB_NAME} moveit_collision_detection ${catkin_LIBRARIES} ${LIBFCL_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)

catkin_add_gtest(test_fcl_collision_detection test/test_fcl_collision_detection.cpp)
target_link_libraries(test_fcl_collision_detection  ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})
