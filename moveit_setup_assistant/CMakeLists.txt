cmake_minimum_required(VERSION 2.8.12)
project(moveit_setup_assistant)

add_compile_options(-std=c++11)

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
  srdfdom
)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# Qt Stuff
if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

# Support new yaml-cpp API.
find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

find_library(YAML yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_tools
  CATKIN_DEPENDS
    moveit_ros_planning
    moveit_ros_visualization
)

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set(HEADERS
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
  src/widgets/author_information_widget.h
)

# Tools Library
add_library(${PROJECT_NAME}_tools
  src/tools/compute_default_collisions.cpp
  src/tools/moveit_config_data.cpp
)
set_target_properties(${PROJECT_NAME}_tools PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${PROJECT_NAME}_tools
  ${YAML}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${QT_LIBRARIES}
)

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
  src/widgets/author_information_widget.cpp
  ${HEADERS}
)
set_target_properties(${PROJECT_NAME}_widgets PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${PROJECT_NAME}_widgets
  ${PROJECT_NAME}_tools
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# executables
add_executable(${PROJECT_NAME} src/setup_assistant_main.cpp)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_widgets ${PROJECT_NAME}_tools
  ${QT_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} log4cxx)

add_executable(${PROJECT_NAME}_updater src/collisions_updater.cpp )
target_link_libraries(${PROJECT_NAME}_updater
  ${PROJECT_NAME}_tools ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${PROJECT_NAME}_updater
                      PROPERTIES OUTPUT_NAME collisions_updater
                      PREFIX "")

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_widgets ${PROJECT_NAME}_tools ${PROJECT_NAME}_updater
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY templates DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
