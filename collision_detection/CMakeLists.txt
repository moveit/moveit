add_library(collision_detection
  src/collision_world.cpp 
  src/collision_robot.cpp
  src/collision_matrix.cpp
  src/collision_tools.cpp
  src/allvalid/collision_robot.cpp
  src/allvalid/collision_world.cpp
)

target_link_libraries(collision_detection planning_models ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS collision_detection
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)
