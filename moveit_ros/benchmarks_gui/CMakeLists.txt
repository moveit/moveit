cmake_minimum_required(VERSION 2.8.12)
project(moveit_ros_benchmarks_gui)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread program_options date_time system filesystem)

find_package(PkgConfig REQUIRED)

pkg_check_modules(OGRE OGRE)
link_directories(${OGRE_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning
  moveit_ros_warehouse
  moveit_ros_benchmarks
  moveit_ros_visualization
  eigen_conversions
  interactive_markers
  rviz
  tf
  roscpp
  rosconsole
)

pkg_search_module(EIGEN3 REQUIRED eigen3) 

if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
else()
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
    INCLUDE_DIRS 
        ${EIGEN3_INCLUDE_DIRS} 
)

include_directories(include)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

set(HEADERS
  include/benchmark_processing_thread.h
  include/frame_marker.h
  include/job_processing.h
  include/main_window.h
  include/trajectory.h
  include/ui_utils.h
)
qt_wrap_ui(UIC_FILES
  src/ui/main_window.ui
  src/ui/run_benchmark_dialog.ui
  src/ui/robot_loader.ui
  src/ui/bounding_box_goals.ui
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(SYSTEM
                    ${QT_INCLUDE_DIR}
                    ${OGRE_INCLUDE_DIRS})

set(SOURCES
  src/main.cpp
  src/main_window.cpp
  src/job_processing.cpp
  src/benchmark_processing_thread.cpp
  src/ui_utils.cpp
  src/frame_marker.cpp
  src/trajectory.cpp
  src/tab_states_and_goals.cpp
  src/tab_trajectories.cpp
)

add_executable(moveit_benchmark_gui ${SOURCES} ${HEADERS} ${UIC_FILES})

target_link_libraries(moveit_benchmark_gui
  ${catkin_LIBRARIES}
  ${rviz_DEFAULT_PLUGIN_LIBRARIES}
  ${OGRE_LIBRARIES}
  ${QT_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(
  TARGETS
    moveit_benchmark_gui
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION include)
