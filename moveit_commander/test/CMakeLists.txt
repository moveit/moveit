if (CATKIN_ENABLE_TESTING)
  find_package(moveit_resources)
  find_package(rostest)

  add_rostest(python_moveit_commander.test)
  add_rostest(python_moveit_commander_ns.test)
endif()

install(PROGRAMS python_moveit_commander.py
                 python_moveit_commander_ns.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)

install(FILES python_moveit_commander.test
              python_moveit_commander_ns.test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)
