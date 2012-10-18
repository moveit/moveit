add_library(collision_detection_fcl
  src/collision_common.cpp
  src/collision_robot.cpp
  src/collision_world.cpp
)

target_link_libraries(collision_detection_fcl collision_detection ${catkin_LIBRARIES} ${FCL_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS collision_detection_fcl
  LIBRARY DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)

catkin_add_gtest(test_fcl_collision_detection test/test_fcl_collision_detection.cpp)
target_link_libraries(test_fcl_collision_detection collision_detection_fcl ${Boost_LIBRARIES})
