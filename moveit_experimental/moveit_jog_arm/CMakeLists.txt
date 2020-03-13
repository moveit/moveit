cmake_minimum_required(VERSION 3.1.3)
project(moveit_jog_arm)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(LIBRARY_NAME jog_arm_cpp_api)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rosparam_shortcuts
  sensor_msgs
  std_msgs
  trajectory_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${LIBRARY_NAME}
  CATKIN_DEPENDS
    control_msgs
    geometry_msgs
    moveit_msgs
    moveit_ros_planning_interface
    rosparam_shortcuts
    sensor_msgs
    std_msgs
    trajectory_msgs
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

#########################################
## A library providing a C++ interface ##
#########################################

add_library(${LIBRARY_NAME} SHARED
  src/collision_check_thread.cpp
  src/jog_calcs.cpp
  src/jog_cpp_interface.cpp
  src/jog_interface_base.cpp
  src/low_pass_filter.cpp
)
add_dependencies(${LIBRARY_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${LIBRARY_NAME}
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

# An example of using the C++ library
add_executable(cpp_interface_example
  src/cpp_interface_example/cpp_interface_example.cpp
)
add_dependencies(cpp_interface_example ${catkin_EXPORTED_TARGETS})
target_link_libraries(cpp_interface_example
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${LIBRARY_NAME}
)

############################
## ROS message-based node ##
############################

add_executable(jog_server
  src/collision_check_thread.cpp
  src/jog_calcs.cpp
  src/jog_interface_base.cpp
  src/jog_ros_interface.cpp
  src/jog_server.cpp
  src/low_pass_filter.cpp
)
add_dependencies(jog_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(jog_server ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

################################################
## An example of converting joystick commands ##
################################################

add_executable(spacenav_to_twist
  src/teleop_examples/spacenav_to_twist.cpp
)
add_dependencies(spacenav_to_twist ${catkin_EXPORTED_TARGETS})
target_link_libraries(spacenav_to_twist ${catkin_LIBRARIES})

##################
## INSTALLATION ##
##################

install(
  TARGETS
    ${LIBRARY_NAME}
    jog_server
    spacenav_to_twist
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## TESTING ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(ros_pytest REQUIRED)

  # Python integration tests
  add_rostest(test/launch/jog_arm_halt_msg_test.test)
  add_rostest(test/launch/jog_arm_msg_reception_test.test)
  add_rostest(test/launch/jog_arm_vel_accel_limits_test.test)
  add_rostest(test/launch/jog_arm_drift_dimensions_service_test.test)

  # jog_cpp_interface
  add_rostest_gtest(jog_cpp_interface_test
    test/jog_cpp_interface_test.test
    test/jog_cpp_interface_test.cpp
  )
  target_link_libraries(jog_cpp_interface_test
    ${LIBRARY_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
endif()
