set(MOVEIT_LIB_NAME moveit_warehouse)

add_library(${MOVEIT_LIB_NAME} 
  src/moveit_message_storage.cpp
  src/planning_scene_storage.cpp
  src/constraints_storage.cpp
  src/state_storage.cpp
  src/warehouse_connector.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(warehouse_connector_test test/warehouse_connector_test.cpp)
target_link_libraries(warehouse_connector_test ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})

add_executable(moveit_warehouse_broadcast src/broadcast.cpp)
target_link_libraries(moveit_warehouse_broadcast ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})

add_executable(moveit_save_to_warehouse src/save_to_warehouse.cpp)
target_link_libraries(moveit_save_to_warehouse ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} moveit_save_to_warehouse moveit_warehouse_broadcast
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
