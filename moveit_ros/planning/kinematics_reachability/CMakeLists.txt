set(MOVEIT_LIB_NAME moveit_kinematics_reachability)

set(workspace_gui_SRCS
  src/main_gui.cpp
  src/mainwindow.cpp
  src/kinematics_thread.cpp
)

set(workspace_gui_HDRS
  include/moveit/kinematics_reachability/mainwindow.h
  include/moveit/kinematics_reachability/kinematics_thread.h
)

set(workspace_gui_UIS
  src/mainwindow.ui
)



set(workspace_gui_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

qt4_wrap_cpp(workspace_gui_MOCS ${workspace_gui_HDRS})
qt4_wrap_ui(workspace_gui_UIS_H ${workspace_gui_UIS})

add_library(moveit_kinematics_thread src/kinematics_thread.cpp)


include_directories(${workspace_gui_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_executable(workspace_gui ${workspace_gui_SRCS} ${workspace_gui_MOCS} ${workspace_gui_UIS_H})
target_link_libraries(workspace_gui ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} moveit_kinematics_thread ${MOVEIT_LIB_NAME})

add_library(${MOVEIT_LIB_NAME} src/kinematics_reachability.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} moveit_planning_scene_monitor ${catkin_LIBRARIES})

add_executable(kinematics_reachability src/main.cpp)
target_link_libraries(kinematics_reachability ${MOVEIT_LIB_NAME} moveit_planning_scene_monitor ${catkin_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)



#set(MOVEIT_LIB_NAME moveit_kinematics_reachability)
#cmake_minimum_required(VERSION 2.8.3)
#project(kinematics_reachability)
#find_package(Qt4 REQUIRED)
#include(${QT_USE_FILE})
#add_definitions(-DQT_NO_KEYWORDS)
#include_directories(${QT_INCLUDE_DIR})
#include_directories(${CMAKE_BINARY_DIR})

## Header files that need Qt Moc pre-processing for use with Qt signals, etc:
#set( headers
#  src/mainwindow.h 
#  src/kinematics_thread.h
#)
## Convert the Qt Signals and Slots for QWidget events
#qt4_wrap_cpp(moc_sources ${headers})

## Main Widgets Library - all screens (navigation options)
#add_library(${MOVEIT_LIB_NAME}
#  src/kinematics_reachability.cpp
#)
#target_link_libraries(${PROJECT_NAME}_widgets ${QT_LIBRARIES} ${catkin_LIBRARIES})

## Main Widgets Library - all screens (navigation options)
#add_library(${PROJECT_NAME}_widgets
#  src/main_gui.cpp 
#  src/mainwindow.cpp 
#  src/kinematics_thread.cpp
#)
#target_link_libraries(${PROJECT_NAME}_widgets ${QT_LIBRARIES} ${catkin_LIBRARIES})

#qt4_wrap_ui(UIC_FILES
#  src/mainwindow.ui
#)

## Source files for the exectuable
#set( sources
#  src/main.cpp
#)

## Link libraries
#add_executable(${PROJECT_NAME} ${sources} ${headers} ${moc_sources})
#target_link_libraries(${PROJECT_NAME} 
#  ${PROJECT_NAME}_widgets 
#  ${QT_LIBRARIES} ${catkin_LIBRARIES} log4cxx)

#install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_widgets 
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(DIRECTORY include/ DESTINATION include)

#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY templates DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
