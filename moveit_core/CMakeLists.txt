cmake_minimum_required(VERSION 2.8.3)
project(moveit_core)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(PkgConfig REQUIRED)
find_package(catkin REQUIRED 
COMPONENTS
  moveit_msgs
  resource_retriever
  urdfdom
  srdfdom
  orocos_kdl
  kdl_parser
  geometric_shapes
  shape_tools
  eigen_stl_containers 
  eigen_conversions
  random_numbers
  console_bridge
  visualization_msgs
  rostime
)
pkg_check_modules(FCL REQUIRED fcl)
find_package(octomap REQUIRED)

# Sets a variable for passing this along in catkin_package()
find_library(LIBFCL_LIBRARIES NAMES ${FCL_LIBRARIES})

# octomap does not follow the find_package convention, 
# so we have to find the full paths and store in a variable; 
# you SHOULD NOT use OCTOMAP_LIBRARIES as it will muck up dependent packages.
# Also, this way we can manually pass it along in catkin_package()
find_library(LIBOCTOMAP_LIBRARIES NAMES ${OCTOMAP_LIBRARIES})

#message( STATUS "FCL found: " ${FCL_LIBRARIES} " at " ${LIBFCL_LIBRARIES})
#message( STATUS "OCTOMAP found: " ${OCTOMAP_LIBRARIES} " at " ${LIBOCTOMAP_LIBRARIES})
unset( OCTOMAP_LIBRARIES )
unset( FCL_LIBRARIES )

set(THIS_PACKAGE_INCLUDE_DIRS
    collision_detection/include
    collision_detection_fcl/include
    collision_distance_field/include
    constraint_samplers/include
    controller_manager/include
    distance_field/include
    dynamics_solver/include
    kinematics_base/include
    kinematics_cache/include
    kinematics_metrics/include
    kinematic_model/include
    kinematic_state/include
    kinematics_planner/include
    kinematic_constraints/include
    planning_interface/include
    planning_request_adapter/include
    planning_scene/include
    sensor_manager/include
    trajectory_processing/include
)

catkin_package(
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
    ${FCL_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
  LIBRARIES
    moveit_kinematic_model
    moveit_kinematic_state
    moveit_collision_detection
    moveit_collision_detection_fcl
    moveit_kinematic_constraints
    moveit_planning_scene
    moveit_constraint_samplers
    moveit_planning_request_adapter
    moveit_trajectory_processing
    moveit_distance_field
    moveit_collision_distance_field
    moveit_kinematics_metrics
    moveit_dynamics_solver
    moveit_kinematics_cache
    moveit_kinematics_planner
    ${LIBFCL_LIBRARIES}
    ${LIBOCTOMAP_LIBRARIES}
  CATKIN_DEPENDS
    geometric_shapes
    shape_tools
    eigen_stl_containers
    eigen_conversions
    random_numbers
    console_bridge
    moveit_msgs
    srdfdom
  DEPENDS
    Boost
    Eigen
    urdfdom
    )

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS}
                           ${Boost_INCLUDE_DIRS}
			   )

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} 
                    ${catkin_INCLUDE_DIRS} 
                    ${FCL_INCLUDE_DIRS}
                    )

link_directories(${FCL_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(kinematics_base)
add_subdirectory(controller_manager)
add_subdirectory(sensor_manager)
add_subdirectory(kinematic_model)
add_subdirectory(kinematic_state)
add_subdirectory(collision_detection)
add_subdirectory(collision_detection_fcl)
add_subdirectory(kinematic_constraints)
add_subdirectory(planning_scene)
add_subdirectory(constraint_samplers)
add_subdirectory(planning_interface)
add_subdirectory(planning_request_adapter)
add_subdirectory(trajectory_processing)
add_subdirectory(distance_field)
add_subdirectory(collision_distance_field)
add_subdirectory(kinematics_metrics)
add_subdirectory(dynamics_solver)
add_subdirectory(kinematics_cache)
add_subdirectory(kinematics_planner)
