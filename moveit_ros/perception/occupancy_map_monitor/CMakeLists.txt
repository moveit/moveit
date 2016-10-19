set(MOVEIT_LIB_NAME moveit_occupancy_map_monitor)

add_library(${MOVEIT_LIB_NAME}
  src/occupancy_map_monitor.cpp
  src/occupancy_map_updater.cpp
  )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION 0.7.3)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

add_executable(moveit_occupancy_map_server src/occupancy_map_server.cpp)
target_link_libraries(moveit_occupancy_map_server ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
