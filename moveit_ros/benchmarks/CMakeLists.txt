cmake_minimum_required(VERSION 2.8.12)
project(moveit_ros_benchmarks)

set(MOVEIT_LIB_NAME moveit_ros_benchmarks)

add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED filesystem)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning
  moveit_ros_warehouse
  roscpp
)

catkin_package(
  LIBRARIES ${MOVEIT_LIB_NAME}
  CATKIN_DEPENDS
    moveit_ros_planning
    moveit_ros_warehouse
    roscpp
  DEPENDS
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/include)
link_directories(${catkin_LIBRARY_DIRS})

add_library(${MOVEIT_LIB_NAME} src/BenchmarkOptions.cpp
                               src/BenchmarkExecutor.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_run_benchmark src/RunBenchmark.cpp)
target_link_libraries(moveit_run_benchmark ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
  TARGETS
    ${MOVEIT_LIB_NAME} moveit_run_benchmark
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(PROGRAMS scripts/moveit_benchmark_statistics.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY examples/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples)
