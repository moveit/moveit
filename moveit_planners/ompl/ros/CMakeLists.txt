cmake_minimum_required(VERSION 2.8.3)
project(moveit_ompl_planners_ros_plugin)

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(catkin COMPONENTS
  moveit_ros_planning
  moveit_ompl_planners_core
  roscpp
  rosconsole
  pluginlib
  tf
  dynamic_reconfigure
  REQUIRED)

generate_dynamic_reconfigure_options("ompl_interface_ros/cfg/OMPLParamsDR.cfg")

catkin_package(
  LIBRARIES
    moveit_ompl_interface_ros
  INCLUDE_DIRS
    ompl_interface_ros/include
  DEPENDS 
    moveit_ompl_planners_core
    moveit_ros_planning
    )

include_directories(ompl_interface_ros/include 
                    ${catkin_INCLUDE_DIRS} 
                    ${EIGEN_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})

# for buildspace as a single package
include_directories(cfg/cpp)
# for buildspace in a workspace
include_directories(${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/stamps/${PROJECT_NAME}/cfg/cpp)

add_subdirectory(ompl_interface_ros)

install(FILES ompl_interface_ros_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
