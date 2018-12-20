if (CATKIN_ENABLE_TESTING)
  find_package(moveit_resources REQUIRED)
  find_package(rostest REQUIRED)

  add_rostest(python_moveit_commander.test)
  add_rostest(python_moveit_commander_ns.test)
endif()
