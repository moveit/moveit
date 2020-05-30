cmake_minimum_required(VERSION 3.1.3)
project(moveit_jog_arm)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(LIBRARY_NAME moveit_jog_arm)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  nodelet
  rosparam_shortcuts
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${LIBRARY_NAME}
    ${LIBRARY_NAME}_nodelet
  CATKIN_DEPENDS
    control_msgs
    geometry_msgs
    moveit_msgs
    moveit_ros_planning_interface
    nodelet
    rosparam_shortcuts
    sensor_msgs
    std_msgs
    std_srvs
    trajectory_msgs
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

#########################################
## A library providing a C++ interface ##
#########################################

add_library(${LIBRARY_NAME} SHARED
  src/collision_check.cpp
  src/jog_calcs.cpp
  src/jog_arm.cpp
  src/low_pass_filter.cpp
)
add_dependencies(${LIBRARY_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${LIBRARY_NAME}
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

#####################
## Jog Arm Nodelet ##
#####################

add_library(${LIBRARY_NAME}_nodelet SHARED
  src/jog_arm_nodelet.cpp
)
add_dependencies(${LIBRARY_NAME}_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(${LIBRARY_NAME}_nodelet
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
  ${LIBRARY_NAME}
)

#########################################
## An example of using the C++ library ##
#########################################

# An example of using the C++ library
add_executable(cpp_interface_example
  src/cpp_interface_example/cpp_interface_example.cpp
)
add_dependencies(cpp_interface_example ${catkin_EXPORTED_TARGETS})
target_link_libraries(cpp_interface_example
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
  ${LIBRARY_NAME}
)

###############################
## An example of the Nodelet ##
###############################

# An example of using the C++ library
add_library(nodelet_example SHARED
  src/nodelet_example/nodelet_example.cpp
)
add_dependencies(nodelet_example ${catkin_EXPORTED_TARGETS})
target_link_libraries(nodelet_example
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

################################################
## An example of converting joystick commands ##
################################################

add_library(spacenav_to_twist SHARED
  src/teleop_examples/spacenav_to_twist.cpp
)
add_dependencies(spacenav_to_twist ${catkin_EXPORTED_TARGETS})
target_link_libraries(spacenav_to_twist
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

##################
## INSTALLATION ##
##################

install(
  TARGETS
    ${LIBRARY_NAME}
    ${LIBRARY_NAME}_nodelet
    cpp_interface_example
    nodelet_example
    spacenav_to_twist
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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
    ${Eigen_LIBRARIES}
    ${Boost_LIBRARIES}
  )
endif()
