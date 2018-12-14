if(CATKIN_ENABLE_TESTING)
  find_package(moveit_resources REQUIRED)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_kinematics_plugin all.test test_kinematics_plugin.cpp)
  target_link_libraries(test_kinematics_plugin ${catkin_LIBRARIES})
  target_compile_options(test_kinematics_plugin PRIVATE -Wno-deprecated-declarations)
endif()
