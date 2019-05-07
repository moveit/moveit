set(JOG_ARM_NAME jog_arm)

add_executable(${JOG_ARM_NAME}_server
  src/jog_arm/collision_check_thread.cpp
  src/jog_arm/jog_arm_server.cpp
  src/jog_arm/jog_calcs.cpp
  src/jog_arm/jog_ros_interface.cpp
  src/jog_arm/low_pass_filter.cpp
)
add_dependencies(${JOG_ARM_NAME}_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(${JOG_ARM_NAME}_server ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(${JOG_ARM_NAME}_spacenav_to_twist
  src/jog_arm/teleop_examples/spacenav_to_twist.cpp
)
add_dependencies(${JOG_ARM_NAME}_spacenav_to_twist ${catkin_EXPORTED_TARGETS})
target_link_libraries(${JOG_ARM_NAME}_spacenav_to_twist ${catkin_LIBRARIES})

install(TARGETS ${JOG_ARM_NAME}_server ${JOG_ARM_NAME}_spacenav_to_twist
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(ros_pytest REQUIRED)
  add_rostest(test/launch/jog_arm_integration_test.test)
endif()
