set(MOVEIT_LIB_NAME moveit_default_planning_request_adapter_plugins)

set(SOURCE_FILES
        src/empty.cpp
        src/fix_start_state_bounds.cpp
        src/fix_start_state_collision.cpp
        src/fix_start_state_path_constraints.cpp
        src/fix_workspace_bounds.cpp
        src/add_time_parameterization.cpp
        src/add_iterative_spline_parameterization.cpp
        src/chomp_optimizer_adapter.cpp)

find_package(catkin REQUIRED COMPONENTS
        cmake_modules
        roscpp
        moveit_core
        pluginlib
        chomp_motion_planner
        moveit_experimental
        )

find_package(Eigen3 REQUIRED)


include_directories(../../../moveit_planners
        ../../../moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner
        ../../../moveit_planners/chomp/chomp_motion_planner/include/
        ../../../moveit_planners/chomp/chomp_motion_planner/
        ../../../moveit_experimental/collision_distance_field/include/moveit/collision_distance_field
        ../../../moveit_experimental/collision_distance_field/include/
        ../../../moveit_experimental/collision_distance_field/)

add_library(${MOVEIT_LIB_NAME} ${SOURCE_FILES})
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_list_request_adapter_plugins src/list.cpp)
target_link_libraries(moveit_list_request_adapter_plugins ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} moveit_list_request_adapter_plugins
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})