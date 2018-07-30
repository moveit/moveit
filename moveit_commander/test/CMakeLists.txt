if (CATKIN_ENABLE_TESTING)
  find_package(moveit_resources)
  find_package(rostest)

  add_rostest(python_moveit_commander.test)
  add_rostest(python_moveit_commander_ns.test)
endif()
