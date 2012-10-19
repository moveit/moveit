set(MOVEIT_LIB_NAME moveit_ompl_interface_ros)

add_library(${MOVEIT_LIB_NAME} src/ompl_interface_ros.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ompl_planner src/ompl_planner.cpp)
target_link_libraries(ompl_planner ${MOVEIT_LIB_NAME})

add_library(moveit_ompl_planner_plugin src/ompl_plugin.cpp)
target_link_libraries(moveit_ompl_planner_plugin ${MOVEIT_LIB_NAME})

#add dynamic reconfigure api
#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()
