cmake_minimum_required(VERSION 2.8.3)
project(stomp_moveit)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  stomp_core
  cmake_modules
  pluginlib
  industrial_collision_detection
)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_definitions("-std=c++11")


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS moveit_ros_planning moveit_core stomp_core cmake_modules pluginlib roscpp
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/stomp_optimization_task.cpp
  src/stomp_planner.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# planner manager plugin
add_library(${PROJECT_NAME}_planner_manager
  src/stomp_planner_manager.cpp)
target_link_libraries(${PROJECT_NAME}_planner_manager ${PROJECT_NAME} ${catkin_LIBRARIES})

# cost function plugin(s)
add_library(${PROJECT_NAME}_cost_functions
  src/cost_functions/tool_goal_pose.cpp
  src/cost_functions/obstacle_avoidance.cpp
  src/cost_functions/collision_check.cpp
 )
target_link_libraries(${PROJECT_NAME}_cost_functions ${catkin_LIBRARIES})

# filter plugin(s)
add_library(${PROJECT_NAME}_noisy_filters
  src/noisy_filters/joint_limits.cpp  
  src/noisy_filters/multi_trajectory_visualization.cpp
  src/noisy_filters/underconstrained_goal.cpp
)

target_link_libraries(${PROJECT_NAME}_noisy_filters ${catkin_LIBRARIES})

# update plugin(s)
add_library(${PROJECT_NAME}_update_filters
  src/update_filters/polynomial_smoother.cpp
  src/update_filters/constrained_cartesian_goal.cpp
  src/update_filters/trajectory_visualization.cpp
  src/update_filters/control_cost_projection.cpp
  src/update_filters/update_logger.cpp
 )
target_link_libraries(${PROJECT_NAME}_update_filters ${catkin_LIBRARIES})

# noise generator plugin(s)
add_library(${PROJECT_NAME}_noise_generators
  src/noise_generators/normal_distribution_sampling.cpp
 )
target_link_libraries(${PROJECT_NAME}_noise_generators ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############

