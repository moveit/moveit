cmake_minimum_required(VERSION 2.8.12)
project(moveit_ros_occupancy_map_monitor)
set(MOVEIT_LIB_NAME ${PROJECT_NAME})

add_compile_options(-std=c++14)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)

if(APPLE)
  find_package(X11 REQUIRED)
endif(APPLE)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_msgs
  pluginlib
)

find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${MOVEIT_LIB_NAME}
  CATKIN_DEPENDS
    moveit_core
    moveit_msgs
  DEPENDS
    EIGEN3
    OCTOMAP
)

include_directories(include
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    )
include_directories(SYSTEM
                    ${EIGEN3_INCLUDE_DIRS}
                    ${X11_INCLUDE_DIR}
                    )

add_library(${MOVEIT_LIB_NAME}
  src/occupancy_map_monitor.cpp
  src/occupancy_map_updater.cpp
  )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_ros_occupancy_map_server src/occupancy_map_server.cpp)
target_link_libraries(moveit_ros_occupancy_map_server ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} moveit_ros_occupancy_map_server
  LIBRARY DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
