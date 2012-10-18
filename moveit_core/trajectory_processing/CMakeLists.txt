add_library(trajectory_processing
  src/iterative_smoother.cpp
  src/trajectory_tools.cpp
)
target_link_libraries(trajectory_processing planning_scene ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS trajectory_processing
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)
