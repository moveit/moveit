if (CATKIN_ENABLE_TESTING)
  find_package(moveit_resources REQUIRED)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_cleanup cleanup.test cleanup.cpp)
  target_link_libraries(test_cleanup moveit_move_group_interface)

  add_rostest(python_move_group.test)
  add_rostest(python_move_group_ns.test)
  add_rostest(robot_state_update.test)
endif()

install(PROGRAMS python_move_group.py python_move_group_ns.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)
install(FILES python_move_group.test python_move_group_ns.test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)
