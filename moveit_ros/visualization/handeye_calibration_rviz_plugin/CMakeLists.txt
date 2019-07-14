find_package(OpenCV REQUIRED) 

set(HEADERS
  include/moveit/handeye_calibration_rviz_plugin/handeye_calibration_gui.h
  include/moveit/handeye_calibration_rviz_plugin/handeye_target_widget.h
  include/moveit/handeye_calibration_rviz_plugin/handeye_context_widget.h
  include/moveit/handeye_calibration_rviz_plugin/handeye_control_widget.h
)

#catkin_lint: ignore_once missing_directory
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Plugin Source
set(SOURCE_FILES
  src/handeye_calibration_gui.cpp
  src/handeye_target_widget.cpp
  src/handeye_context_widget.cpp
  src/handeye_control_widget.cpp
)

set(MOVEIT_LIB_NAME moveit_handeye_calibration_rviz_plugin)
add_library(${MOVEIT_LIB_NAME}_core ${SOURCE_FILES} ${HEADERS})
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}_core
  moveit_rviz_plugin_render_tools
  moveit_planning_scene_rviz_plugin_core
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${OGRE_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES})

add_library(${MOVEIT_LIB_NAME} src/plugin_init.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(TARGETS ${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
