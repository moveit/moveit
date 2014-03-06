set(MOVEIT_LIB_NAME moveit_pick_place_planner)

add_library(${MOVEIT_LIB_NAME}
  src/pick_place_params.cpp
  src/manipulation_pipeline.cpp
  src/reachable_valid_pose_filter.cpp 
  src/approach_and_translate_stage.cpp
  src/plan_stage.cpp 
  src/pick_place.cpp
  src/pick.cpp
  src/place.cpp
  )

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
