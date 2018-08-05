set(MOVEIT_LIB_NAME moveit_default_planning_request_adapter_plugins)

set(SOURCE_FILES
  src/empty.cpp
  src/fix_start_state_bounds.cpp
  src/fix_start_state_collision.cpp
  src/fix_start_state_path_constraints.cpp
  src/fix_workspace_bounds.cpp
  src/add_time_parameterization.cpp
  src/add_iterative_spline_parameterization.cpp
  src/stomp_smoothing_adapter.cpp)

find_package(catkin REQUIRED COMPONENTS
        cmake_modules
        roscpp
        moveit_core
        pluginlib
        chomp_motion_planner
        moveit_experimental
        )

find_package(Eigen3 REQUIRED)

find_package(stomp_moveit)
find_package(stomp_core)

find_library(stomp_core PATHS ../../../../industrial_moveit/stomp_core )

catkin_package(
        CATKIN_DEPENDS  stomp_core
)

include_directories(../../../moveit_planners
        ../../../moveit_planners/stomp
        ../../../moveit_planners/stomp/include
        ../../../moveit_planners/stomp/include/stomp_moveit
        ../../../../industrial_moveit/stomp_core
        ../../../../industrial_moveit/stomp_core/include/stomp_core
        ../../../../industrial_moveit/stomp_core/include
        )

add_library(${MOVEIT_LIB_NAME} ${SOURCE_FILES})
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} stomp_moveit stomp_core)

add_executable(moveit_list_request_adapter_plugins src/list.cpp)
target_link_libraries(moveit_list_request_adapter_plugins ${catkin_LIBRARIES} ${Boost_LIBRARIES} stomp_core stomp_moveit)

install(TARGETS ${MOVEIT_LIB_NAME} moveit_list_request_adapter_plugins
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

