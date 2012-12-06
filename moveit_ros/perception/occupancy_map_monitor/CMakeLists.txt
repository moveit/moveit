set(MOVEIT_LIB_NAME moveit_occupancy_map_monitor)

# octomap does not follow the find_package convention, 
# so we have to find the full paths, set them to a 
# variable, and NEVER  use OCTOMAP_LIBRARIES.
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
find_library(LIBOCTOMAP_LIBRARIES NAMES ${OCTOMAP_LIBRARIES})
unset( OCTOMAP_LIBRARIES )

add_library(${MOVEIT_LIB_NAME}
  src/occupancy_map_monitor.cpp
  src/point_cloud_occupancy_map_updater.cpp
  src/octomap_markers.cpp
  )
target_link_libraries(${MOVEIT_LIB_NAME} ${LIBOCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} moveit_robot_self_filter)

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

add_executable(moveit_occupancy_map_server src/occupancy_map_server.cpp)
target_link_libraries(moveit_occupancy_map_server ${MOVEIT_LIB_NAME}
  ${LIBOCTOMAP_LIBRARIES} ${Boost_LIBRARIES} moveit_robot_self_filter)
