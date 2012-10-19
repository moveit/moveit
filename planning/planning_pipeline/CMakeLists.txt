add_library(moveit_planning_pipeline src/planning_pipeline.cpp)
target_link_libraries(moveit_planning_pipeline ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS moveit_planning_pipeline LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
