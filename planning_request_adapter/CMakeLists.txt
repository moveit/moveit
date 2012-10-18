
add_library(planning_request_adapter src/planning_request_adapter.cpp)
target_link_libraries(planning_request_adapter planning_scene ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS planning_request_adapter
  LIBRARY DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)
