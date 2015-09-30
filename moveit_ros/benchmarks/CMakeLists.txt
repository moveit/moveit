cmake_minimum_required(VERSION 2.8.12)
project(moveit_ros_benchmarks)

set(MOVEIT_LIB_NAME moveit_benchmarks)

add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED filesystem)

# In ROS Hydro, MoveIt links against version 2 of Boost Filesystem
# Force everything to use this version to prevent compile error
if ($ENV{ROS_DISTRO} STREQUAL "hydro")
  message("Explicitly linking against Boost.Filesystem v2")
  add_definitions(-DUSE_BOOST_FILESYSTEM_V2)
endif()

find_package(catkin COMPONENTS
  moveit_ros_planning
  moveit_ros_warehouse
  roscpp
  REQUIRED)

# catkin_package defines the env vars used in install later
catkin_package(
  INCLUDE_DIRS
    benchmarks/include
  LIBRARIES ${MOVEIT_LIB_NAME}
  CATKIN_DEPENDS
    moveit_ros_planning
    moveit_ros_warehouse
    roscpp
  DEPENDS
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
)

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/include)
link_directories(${catkin_LIBRARY_DIRS})

add_library(${MOVEIT_LIB_NAME} src/BenchmarkOptions.cpp
                               src/BenchmarkExecutor.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_run_benchmark src/RunBenchmark.cpp)
target_link_libraries(moveit_run_benchmark ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
  TARGETS
    ${MOVEIT_LIB_NAME} moveit_run_benchmark
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
