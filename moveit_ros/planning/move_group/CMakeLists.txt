add_executable(move_group_action_server src/move_group.cpp)
target_link_libraries(move_group_action_server plan_execution ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS move_group_action_server RUNTIME DESTINATION bin)
