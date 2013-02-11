cmake_minimum_required(VERSION 2.8.3)
project(moveit_ros_planning)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(MSG_DEPS
  std_msgs
  actionlib_msgs
  sensor_msgs
  geometry_msgs
  shape_msgs
  moveit_msgs)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time program_options signals thread)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_perception
  pluginlib
  actionlib
  roscpp
  rosconsole
  dynamic_reconfigure
  message_filters
  orocos_kdl
  srdfdom
  urdf
  tf
  tf_conversions
  genmsg
  ${MSG_DEPS}
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

set(MSG_FILES
  Progress.msg
  WorkspacePoint.msg
  WorkspacePoints.msg)

#add_message_files(DIRECTORY kinematics_reachability/msg FILES ${MSG_FILES})
#generate_messages(DEPENDENCIES ${MSG_DEPS})

generate_dynamic_reconfigure_options(
  "planning_scene_monitor/cfg/PlanningSceneMonitorDynamicReconfigure.cfg"
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
    kdl_kinematics_plugin/include
#    kinematics_reachability/include
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
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    pluginlib
    moveit_core
    moveit_ros_perception
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} 
                    ${catkin_INCLUDE_DIRS}
		    )
include_directories(SYSTEM
                    ${EIGEN_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    )

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(rdf_loader)
add_subdirectory(kdl_kinematics_plugin)
add_subdirectory(kinematics_plugin_loader)
add_subdirectory(robot_model_loader)
add_subdirectory(constraint_sampler_manager_loader)
add_subdirectory(planning_pipeline)
add_subdirectory(planning_request_adapter_plugins)
add_subdirectory(planning_scene_monitor)
add_subdirectory(planning_scene_monitor_tools)
add_subdirectory(trajectory_execution_manager)
add_subdirectory(plan_execution)
#add_subdirectory(kinematics_reachability)

install(
  FILES
    planning_request_adapters_plugin_description.xml
    kdl_kinematics_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})
