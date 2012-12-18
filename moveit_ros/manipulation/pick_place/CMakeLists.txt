set(MOVEIT_LIB_NAME moveit_pick_place_planner)

add_library(${MOVEIT_LIB_NAME}
  src/grasp_filter.cpp
  src/reachable_valid_grasp_filter.cpp
  src/pick_place.cpp
  )

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
