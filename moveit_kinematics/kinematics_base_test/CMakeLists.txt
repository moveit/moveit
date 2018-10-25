cmake_minimum_required(VERSION 2.8.3)
project(kinematics_base_test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  rostest
  rosunit
  moveit_ros_planning
  moveit_kinematics
)

find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Testing ##
#############

add_rostest_gtest(${PROJECT_NAME}_utest launch/test_kinematics_plugin.launch src/test_kinematics_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_utest ${catkin_LIBRARIES} ${boost_LIBRARIES})

#############
## Install ##
#############
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
