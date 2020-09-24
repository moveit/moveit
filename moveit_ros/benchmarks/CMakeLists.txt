cmake_minimum_required(VERSION 3.1.3)
project(moveit_ros_benchmarks)

set(MOVEIT_LIB_NAME moveit_ros_benchmarks)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED filesystem)

find_package(catkin REQUIRED COMPONENTS
  tf2_eigen
  moveit_ros_planning
  moveit_ros_warehouse
  roscpp
  pluginlib
)

catkin_package(
  LIBRARIES ${MOVEIT_LIB_NAME}
  CATKIN_DEPENDS
    tf2_eigen
    moveit_ros_planning
    moveit_ros_warehouse
    roscpp
  DEPENDS
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${MOVEIT_LIB_NAME} src/BenchmarkOptions.cpp
                               src/BenchmarkExecutor.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_run_benchmark src/RunBenchmark.cpp)
target_link_libraries(moveit_run_benchmark ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
  TARGETS
    ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  TARGETS
    moveit_run_benchmark
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

catkin_install_python(
  PROGRAMS scripts/moveit_benchmark_statistics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY examples/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples)
