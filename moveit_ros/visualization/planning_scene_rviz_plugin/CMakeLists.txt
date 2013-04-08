
# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES include/moveit/planning_scene_rviz_plugin/planning_scene_display.h)

set(MOVEIT_LIB_NAME moveit_planning_scene_rviz_plugin)
add_library(${MOVEIT_LIB_NAME}_core
  src/planning_scene_display.cpp
  src/background_processing.cpp
  ${MOC_SOURCES})
target_link_libraries(${MOVEIT_LIB_NAME}_core moveit_rviz_plugin_render_tools ${catkin_LIBRARIES} ${OGRE_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES})

add_library(${MOVEIT_LIB_NAME} src/plugin_init.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${MOVEIT_LIB_NAME}_core ${MOVEIT_LIB_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
