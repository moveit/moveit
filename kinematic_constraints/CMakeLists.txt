add_library(kinematic_constraints
  src/kinematic_constraint.cpp
  src/utils.cpp)

target_link_libraries(kinematic_constraints planning_models collision_detection_fcl ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS kinematic_constraints
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

catkin_add_gtest(test_constraints test/test_constraints.cpp)
target_link_libraries(test_constraints kinematic_constraints)
