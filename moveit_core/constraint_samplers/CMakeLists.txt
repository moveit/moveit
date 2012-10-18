
add_library(constraint_samplers
  src/constraint_sampler.cpp
  src/default_constraint_samplers.cpp
  src/union_constraint_sampler.cpp
  src/constraint_sampler_manager.cpp
  src/constraint_sampler_tools.cpp
)
target_link_libraries(constraint_samplers planning_scene ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS constraint_samplers
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)
