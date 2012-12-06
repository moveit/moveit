set(MOVEIT_LIB_NAME moveit_distance_field)

add_library(${MOVEIT_LIB_NAME}
  src/distance_field.cpp
  src/propagation_distance_field.cpp
  )
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# This line is needed to ensure that messages are done being built before this is built
add_dependencies(${MOVEIT_LIB_NAME} moveit_msgs_gencpp)

install(TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

catkin_add_gtest(test_voxel_grid test/test_voxel_grid.cpp)
target_link_libraries(test_voxel_grid ${MOVEIT_LIB_NAME})

catkin_add_gtest(test_distance_field test/test_distance_field.cpp)
target_link_libraries(test_distance_field ${MOVEIT_LIB_NAME})
