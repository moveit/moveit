cmake_minimum_required(VERSION 2.8.3)
project(moveit_setup_assistant)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(include)

find_package(Boost REQUIRED COMPONENTS thread filesystem system program_options)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning
  moveit_ros_visualization
  rviz
  urdf
  rosconsole
  roscpp
)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# Qt Stuff
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)
include_directories(${QT_INCLUDE_DIR})
include_directories(${CMAKE_BINARY_DIR})

find_library(YAML yaml-cpp REQUIRED)

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set( headers
  src/widgets/navigation_widget.h
  src/widgets/header_widget.h
  src/widgets/setup_assistant_widget.h
  src/widgets/start_screen_widget.h
  src/widgets/planning_groups_widget.h
  src/widgets/double_list_widget.h
  src/widgets/kinematic_chain_widget.h
  src/widgets/group_edit_widget.h
  src/widgets/default_collisions_widget.h
  src/widgets/robot_poses_widget.h
  src/widgets/end_effectors_widget.h
  src/widgets/virtual_joints_widget.h
  src/widgets/passive_joints_widget.h
  src/widgets/configuration_files_widget.h
  src/widgets/setup_screen_widget.h
)

# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(moc_sources ${headers})

# Tools Library
add_library(${PROJECT_NAME}_tools
  src/tools/compute_default_collisions.cpp
  src/tools/moveit_config_data.cpp
  src/tools/srdf_writer.cpp
)
target_link_libraries(${PROJECT_NAME}_tools ${YAML} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Main Widgets Library - all screens (navigation options)
add_library(${PROJECT_NAME}_widgets
  src/widgets/start_screen_widget.cpp
  src/widgets/planning_groups_widget.cpp
  src/widgets/double_list_widget.cpp
  src/widgets/kinematic_chain_widget.cpp
  src/widgets/group_edit_widget.cpp
  src/widgets/default_collisions_widget.cpp
  src/widgets/robot_poses_widget.cpp
  src/widgets/end_effectors_widget.cpp
  src/widgets/virtual_joints_widget.cpp
  src/widgets/passive_joints_widget.cpp
  src/widgets/configuration_files_widget.cpp
  src/widgets/navigation_widget.cpp
  src/widgets/header_widget.cpp
  src/widgets/setup_assistant_widget.cpp
  src/widgets/setup_screen_widget.cpp
)
target_link_libraries(${PROJECT_NAME}_widgets ${PROJECT_NAME}_tools ${QT_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Source files for the exectuable
set( sources
  src/setup_assistant_main.cpp
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}_tools 
  CATKIN_DEPENDS
    moveit_ros_planning
    moveit_ros_visualization
    )

# Link libraries
add_executable(${PROJECT_NAME} ${sources} ${headers} ${moc_sources})
target_link_libraries(${PROJECT_NAME} 
  ${PROJECT_NAME}_widgets ${PROJECT_NAME}_tools 
  ${QT_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} log4cxx)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_widgets ${PROJECT_NAME}_tools
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY templates DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
