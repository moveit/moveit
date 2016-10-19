set(MOVEIT_LIB_NAME moveit_rviz_plugin_render_tools)

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set( headers
  include/moveit/rviz_plugin_render_tools/trajectory_visualization.h
)

# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES ${headers})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(${MOVEIT_LIB_NAME}
  src/render_shapes.cpp
  src/robot_state_visualization.cpp
  src/planning_scene_render.cpp
  src/planning_link_updater.cpp
  src/octomap_render.cpp
  src/trajectory_visualization.cpp
  ${MOC_SOURCES}
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION 0.7.3)

target_link_libraries(${MOVEIT_LIB_NAME}
  ${catkin_LIBRARIES}
  ${OGRE_LIBRARIES}
  ${QT_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
