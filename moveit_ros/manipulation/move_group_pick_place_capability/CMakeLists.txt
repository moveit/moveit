set(MOVEIT_LIB_NAME moveit_move_group_pick_place_capability)

include_directories(src)

add_library(${MOVEIT_LIB_NAME}
  src/pick_place_action_capability.cpp
  )

target_link_libraries(${MOVEIT_LIB_NAME} moveit_pick_place_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/ DESTINATION include)
