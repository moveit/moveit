cmake_minimum_required(VERSION 2.8.12)
project(moveit_ros_planning)

add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED system filesystem date_time program_options signals thread chrono)
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  chomp_motion_planner
  moveit_core
  moveit_msgs
  moveit_ros_perception
  pluginlib
  actionlib
  roscpp
  dynamic_reconfigure
  message_filters
  srdfdom
  urdf
  tf
  tf_conversions
  eigen_conversions
)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

generate_dynamic_reconfigure_options(
  "planning_scene_monitor/cfg/PlanningSceneMonitorDynamicReconfigure.cfg"
  "trajectory_execution_manager/cfg/TrajectoryExecutionDynamicReconfigure.cfg"
  "plan_execution/cfg/PlanExecutionDynamicReconfigure.cfg"
  "plan_execution/cfg/SenseForPlanDynamicReconfigure.cfg")

set(THIS_PACKAGE_INCLUDE_DIRS
    rdf_loader/include
    kinematics_plugin_loader/include
    robot_model_loader/include
    constraint_sampler_manager_loader/include
    planning_pipeline/include
    planning_scene_monitor/include
    trajectory_execution_manager/include
    plan_execution/include
    collision_plugin_loader/include
)

catkin_package(
  LIBRARIES
    moveit_rdf_loader
    moveit_kinematics_plugin_loader
    moveit_robot_model_loader
    moveit_constraint_sampler_manager_loader
    moveit_planning_pipeline
    moveit_trajectory_execution_manager
    moveit_plan_execution
    moveit_planning_scene_monitor
    moveit_collision_plugin_loader
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    chomp_motion_planner
    pluginlib
    moveit_core
    moveit_msgs
    moveit_ros_perception
  DEPENDS
    EIGEN3
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
)
include_directories(SYSTEM
                    ${EIGEN3_INCLUDE_DIRS}
)

add_subdirectory(rdf_loader)
add_subdirectory(collision_plugin_loader)
add_subdirectory(kinematics_plugin_loader)
add_subdirectory(robot_model_loader)
add_subdirectory(constraint_sampler_manager_loader)
add_subdirectory(planning_pipeline)
add_subdirectory(planning_request_adapter_plugins)
add_subdirectory(planning_scene_monitor)
add_subdirectory(planning_components_tools)
add_subdirectory(trajectory_execution_manager)
add_subdirectory(plan_execution)

install(
  FILES
    planning_request_adapters_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})
