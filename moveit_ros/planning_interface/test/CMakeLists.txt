if (CATKIN_ENABLE_TESTING)
  find_package(moveit_resources REQUIRED)
  find_package(rostest REQUIRED)

  add_executable(test_cleanup test_cleanup.cpp)
  target_link_libraries(test_cleanup moveit_move_group_interface)

  add_rostest(python_move_group.test)
  add_rostest(python_move_group_ns.test)
  add_rostest(robot_state_update.test)
  add_rostest(cleanup.test)
endif()
