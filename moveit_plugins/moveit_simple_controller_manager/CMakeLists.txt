cmake_minimum_required(VERSION 2.8.3)
project(moveit_simple_controller_manager)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin COMPONENTS
  moveit_core
  pluginlib
  actionlib
  roscpp
  control_msgs
  REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES
  INCLUDE_DIRS include
  CATKIN_DEPENDS moveit_core actionlib control_msgs
    )

include_directories(include)

add_library(moveit_simple_controller_manager src/moveit_simple_controller_manager.cpp)
target_link_libraries(moveit_simple_controller_manager ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS moveit_simple_controller_manager LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES moveit_simple_controller_manager_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
       )
install(DIRECTORY include/${PROJECT_NAME}/
       DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )
