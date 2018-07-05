cmake_minimum_required(VERSION 2.8.3)
project(stomp_moveit)

add_definitions("-std=c++11")

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  stomp_core
  cmake_modules
  pluginlib
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS moveit_ros_planning moveit_core stomp_core cmake_modules pluginlib roscpp
  DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/stomp_optimization_task.cpp
  src/stomp_planner.cpp
  src/utils/polynomial.cpp
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
  src/cost_functions/collision_check.cpp
  src/cost_functions/obstacle_distance_gradient.cpp
  src/cost_functions/tool_goal_pose.cpp
 )
target_link_libraries(${PROJECT_NAME}_cost_functions ${catkin_LIBRARIES})

# filter plugin(s)
add_library(${PROJECT_NAME}_noisy_filters
  src/noisy_filters/joint_limits.cpp
  src/noisy_filters/multi_trajectory_visualization.cpp
)

target_link_libraries(${PROJECT_NAME}_noisy_filters ${catkin_LIBRARIES})

# update plugin(s)
add_library(${PROJECT_NAME}_update_filters
  src/update_filters/polynomial_smoother.cpp
  src/update_filters/trajectory_visualization.cpp
  src/update_filters/control_cost_projection.cpp
  src/update_filters/update_logger.cpp
  src/utils/polynomial.cpp
  src/update_filters/constrained_cartesian_goal.cpp
 )
target_link_libraries(${PROJECT_NAME}_update_filters ${catkin_LIBRARIES})

# noise generator plugin(s)
add_library(${PROJECT_NAME}_noise_generators
  src/noise_generators/normal_distribution_sampling.cpp
  src/noise_generators/goal_guided_multivariate_gaussian.cpp
 )
target_link_libraries(${PROJECT_NAME}_noise_generators ${catkin_LIBRARIES})

#############
## Install ##
#############
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_planner_manager ${PROJECT_NAME}_cost_functions
  ${PROJECT_NAME}_noisy_filters ${PROJECT_NAME}_update_filters ${PROJECT_NAME}_noise_generators
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES cost_function_plugins.xml noise_generator_plugins.xml update_filter_plugins.xml planner_manager_plugins.xml noisy_filter_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES
    planner_manager_plugins.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
    set(UTEST_SRC_FILES test/utest.cpp
            test/stomp_3dof.cpp)
    catkin_add_gtest(${PROJECT_NAME}_utest ${UTEST_SRC_FILES})
    target_link_libraries(${PROJECT_NAME}_utest ${PROJECT_NAME})

endif()