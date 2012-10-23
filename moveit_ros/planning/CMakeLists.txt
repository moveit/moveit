cmake_minimum_required(VERSION 2.8.3)
project(moveit_ros_planning)

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

find_package(Boost REQUIRED system filesystem date_time signals thread)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin COMPONENTS
  moveit_core
  moveit_ros_perception
  pluginlib
  roscpp
  rosconsole
  dynamic_reconfigure
  message_filters
  srdfdom
  urdf
  tf
  tf_conversions
  REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES
    moveit_robot_model_loader
    moveit_kinematics_plugin_loader
    moveit_kinematics_model_loader
    moveit_default_planning_request_adapter_plugins
  DEPENDS
    pluginlib
    moveit_core)

#get_filename_component(dynamic_reconfigure_SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

include(/opt/ros/groovy/share/dynamic_reconfigure/cmake/rosbuild2.cmake)
#generate_cfg("planning_scene_monitor/cfg/PublishSceneDR.cfg")

include_directories(robot_model_loader/include)
include_directories(kinematics_plugin_loader/include)
include_directories(planning_models_loader/include)
include_directories(controller_manager/include)
include_directories(sensor_manager/include)
include_directories(planning_pipeline/include)
include_directories(planning_scene_monitor/include)
include_directories(trajectory_execution_manager/include)
#include_directories(plan_execution/include)

add_subdirectory(robot_model_loader)
add_subdirectory(kinematics_plugin_loader)
add_subdirectory(planning_models_loader)
add_subdirectory(controller_manager)
add_subdirectory(sensor_manager)
add_subdirectory(planning_pipeline)
add_subdirectory(planning_request_adapter_plugins)
#add_subdirectory(planning_scene_monitor)
#add_subdirectory(planning_scene_monitor_tools)
add_subdirectory(trajectory_execution_manager)
#add_subdirectory(plan_execution)
#add_subdirectory(move_group)



#collision_distance_field_ros
#kdl_kinematics_plugin
#kinematics_cache_ros
#kinematics_planner_ros
#kinematics_reachability
