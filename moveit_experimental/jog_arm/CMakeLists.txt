set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_ros_manipulation
  moveit_ros_move_group
  moveit_ros_planning_interface
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

add_executable(jog_arm_server src/jog_arm/jog_arm_server.cpp)
add_dependencies(jog_arm_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(jog_arm_server ${catkin_LIBRARIES} ${Eigen_LIBRARIES} rosparam_shortcuts)

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

  add_rostest_gtest(compliant_control_utest test/launch/utest.launch ${UTEST_SRC_FILES})
  target_link_libraries(compliant_control_utest ${catkin_LIBRARIES} ${Boost_LIBRARIES} compliant_control)
endif()
