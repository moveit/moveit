cmake_minimum_required(VERSION 2.8.3)
project(moveit_core)

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

find_package(Boost REQUIRED thread)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin COMPONENTS
  moveit_msgs 
  resource_retriever
  urdfdom
  srdfdom
  geometric_shapes
  shape_tools 
  eigen_stl_containers 
  eigen_conversions
  random_numbers
  console_bridge
  visualization_msgs REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package()



add_subdirectory(kinematics_base)
add_subdirectory(planning_models)
#add_subdirectory(collision_detection)
#add_subdirectory(collision_detection_fcl)
#add_subdirectory(kinematic_constraints)
#add_subdirectory(planning_scene)
#add_subdirectory(constraint_samplers)
#add_subdirectory(planning_interface)
#add_subdirectory(planning_request_adapter)
#add_subdirectory(trajectory_processing)
#add_subdirectory(distance_field)
#add_subdirectory(collision_distance_field)

#add_subdirectory(dynamics_solver)
#add_subdirectory(kinematics_cache)
#add_subdirectory(kinematics_constraint_aware)
#add_subdirectory(kinematics_metrics)
#add_subdirectory(kinematics_planner)
