
set(MOVEIT_LIB_NAME moveit_rviz_plugin_render_tools)
add_library(${MOVEIT_LIB_NAME} 
  src/render_shapes.cpp
  src/robot_state_visualization.cpp
  src/planning_scene_render.cpp
  src/planning_link_updater.cpp
  src/octomap_render.cpp
)

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${OGRE_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${MOVEIT_LIB_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
