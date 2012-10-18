add_library(distance_field
  src/distance_field.cpp
  src/propagation_distance_field.cpp
  )
target_link_libraries(distance_field ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS distance_field
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

catkin_add_gtest(test_voxel_grid test/test_voxel_grid.cpp)
target_link_libraries(test_voxel_grid distance_field)

catkin_add_gtest(test_distance_field test/test_distance_field.cpp)
target_link_libraries(test_distance_field distance_field)
