cmake_minimum_required(VERSION 2.8.12)
project(moveit_setup_assistant)

add_compile_options(-std=c++14)
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  # suppress warning in Ogre
  add_compile_options(-Wno-deprecated-register)
endif()

# definition needed for boost/math/constants/constants.hpp included by Ogre to compile
add_definitions(-DBOOST_MATH_DISABLE_FLOAT128)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(include)

find_package(Boost REQUIRED COMPONENTS thread filesystem system program_options)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_visualization
  pluginlib
  rviz
  rosconsole
  roscpp
  urdf
  srdfdom
)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

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
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)
include_directories(${YAMLCPP_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_tools
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    moveit_ros_visualization
    roscpp
    srdfdom
    rviz
    urdf
)

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set(HEADERS
  include/moveit/setup_assistant/navigation_widget.h
  include/moveit/setup_assistant/header_widget.h
  include/moveit/setup_assistant/setup_assistant_widget.h
  include/moveit/setup_assistant/setup_screen_widget.h
  include/moveit/setup_assistant/start_screen_widget.h
  include/moveit/setup_assistant/double_list_widget.h
)

# Tools Library
add_library(${PROJECT_NAME}_tools
  src/tools/compute_default_collisions.cpp
  src/tools/moveit_config_data.cpp
  src/tools/collision_linear_model.cpp
  src/tools/collision_matrix_model.cpp
  src/tools/rotated_header_view.cpp
  include/moveit/setup_assistant/tools/collision_matrix_model.h
)
set_target_properties(${PROJECT_NAME}_tools PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${PROJECT_NAME}_tools
  ${YAMLCPP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${QT_LIBRARIES}
)

# Main Widgets Library - all screens (navigation options)
add_library(${PROJECT_NAME}_widgets
  src/widgets/navigation_widget.cpp
  src/widgets/header_widget.cpp
  src/widgets/setup_assistant_widget.cpp
  src/widgets/setup_screen_widget.cpp
  src/widgets/start_screen_widget.cpp
  src/widgets/double_list_widget.cpp
  ${HEADERS}
)
set_target_properties(${PROJECT_NAME}_widgets PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
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
  LIBRARY DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY templates DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install plugins
install(
  FILES
    plugins/virtual_joints_widget/virtual_joints_widget.xml
    plugins/simulation_widget/simulation_widget.xml
    plugins/ros_controllers_widget/ros_controllers_widget.xml
    plugins/robot_poses_widget/robot_poses_widget.xml
    plugins/planning_groups_widget/planning_groups_widget.xml
    plugins/perception_widget/perception_widget.xml
    plugins/end_effectors_widget/end_effectors_widget.xml
    plugins/default_collisions_widget/default_collisions_widget.xml
    plugins/configuration_files_widget/configuration_files_widget.xml
    plugins/author_information_widget/author_information_widget.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})


# Unit tests
if(CATKIN_ENABLE_TESTING)
  find_package(moveit_resources REQUIRED)
  include_directories(${moveit_resources_INCLUDE_DIRS})
  catkin_add_gtest(test_reading_writing_config test/moveit_config_data_test.cpp)
  target_link_libraries(test_reading_writing_config
    ${PROJECT_NAME}_tools ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${MOVEIT_LIB_NAME})
endif()

add_subdirectory(plugins)
