if (CATKIN_ENABLE_TESTING)
  find_package(rostest)

  # This test fails due to a ROS core bug: https://github.com/ros/ros_comm/pull/871
  #add_rostest_gtest(test_cleanup cleanup.test cleanup.cpp)
  #target_link_libraries(test_cleanup moveit_move_group_interface)

  add_rostest(python_move_group.test)
endif()
