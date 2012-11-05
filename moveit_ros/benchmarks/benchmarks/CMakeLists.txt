
add_executable(moveit_run_benchmark src/run_benchmark.cpp)
target_link_libraries(moveit_run_benchmark  ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_call_benchmark src/call_benchmark.cpp src/plugin_loader.cpp)
target_link_libraries(moveit_call_benchmark  ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS moveit_run_benchmark moveit_call_benchmark
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
