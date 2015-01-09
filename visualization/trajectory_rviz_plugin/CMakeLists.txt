set(MOVEIT_LIB_NAME moveit_trajectory_rviz_plugin)

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set( headers
  include/moveit/trajectory_rviz_plugin/trajectory_display.h
  include/moveit/trajectory_rviz_plugin/trajectory_visualization.h
)

# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES ${headers})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Plugin Source
set(SOURCE_FILES 
  src/trajectory_display.cpp
)

# Trajectory Visualization Helper
add_library(moveit_trajectory_visualization 
  src/trajectory_visualization.cpp)
target_link_libraries(moveit_trajectory_visualization 
  moveit_rviz_plugin_render_tools
  moveit_planning_scene_rviz_plugin_core
  ${catkin_LIBRARIES} ${OGRE_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES})

# Trajectory Display
add_library(${MOVEIT_LIB_NAME}_core ${SOURCE_FILES} ${MOC_SOURCES})
target_link_libraries(${MOVEIT_LIB_NAME}_core
  moveit_trajectory_visualization 
  moveit_rviz_plugin_render_tools
  moveit_planning_scene_rviz_plugin_core
  ${catkin_LIBRARIES} ${OGRE_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES})

# Plugin Initializer
add_library(${MOVEIT_LIB_NAME} src/plugin_init.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
