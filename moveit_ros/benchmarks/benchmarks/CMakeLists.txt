set(MOVEIT_LIB_NAME moveit_benchmark_execution)

add_library(${MOVEIT_LIB_NAME} src/benchmark_execution.cpp src/benchmarks_utils.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION 0.7.3)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_run_benchmark src/run_benchmarks.cpp)
target_link_libraries(moveit_run_benchmark ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
  TARGETS
    ${MOVEIT_LIB_NAME}
    moveit_run_benchmark
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  PROGRAMS scripts/moveit_benchmark_statistics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/ DESTINATION include)
