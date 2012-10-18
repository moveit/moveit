
add_library(planning_scene src/planning_scene.cpp)
target_link_libraries(planning_scene collision_detection_fcl kinematic_constraints ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_planning_scene test/test_planning_scene.cpp)
target_link_libraries(test_planning_scene planning_scene)
