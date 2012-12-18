set(MOVEIT_LIB_NAME moveit_benchmarks_config)

add_library(${MOVEIT_LIB_NAME} src/benchmarks_config.cpp src/benchmarks_utils.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_run_benchmark src/run_benchmark.cpp)
target_link_libraries(moveit_run_benchmark ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_call_benchmark src/call_benchmark.cpp)
target_link_libraries(moveit_call_benchmark ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
  TARGETS
    ${MOVEIT_LIB_NAME}
    moveit_run_benchmark
    moveit_call_benchmark
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION include)
