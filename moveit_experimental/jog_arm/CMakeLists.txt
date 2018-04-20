cmake_minimum_required(VERSION 2.8.3)
project(jog_arm)

add_compile_options(-std=c++11)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_ros_manipulation
  moveit_ros_move_group
  moveit_ros_planning_interface
  cmake_modules
  std_msgs
  tf
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    compliant_control
  CATKIN_DEPENDS
    roscpp
    moveit_ros_manipulation
    moveit_ros_move_group
    moveit_ros_planning_interface
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(compliant_control src/jog_arm/compliant_control/compliant_control.cpp)

add_executable(compliance_test src/jog_arm/compliance_test/compliance_test.cpp)
add_dependencies(compliance_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(compliance_test ${catkin_LIBRARIES} compliant_control)

add_executable(jog_arm_server src/jog_arm/jog_arm_server.cpp src/jog_arm/support/get_ros_params.cpp)
add_dependencies(jog_arm_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(jog_arm_server ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS compliant_control
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  set(UTEST_SRC_FILES test/utest.cpp
      test/compliant_control.cpp)

  add_rostest_gtest(${PROJECT_NAME}_utest test/launch/utest.launch ${UTEST_SRC_FILES})
  target_link_libraries(${PROJECT_NAME}_utest ${catkin_LIBRARIES} ${Boost_LIBRARIES} compliant_control)
endif()