
set(MOVEIT_LIB_NAME moveit_mesh_filter)

add_library(${MOVEIT_LIB_NAME}
  src/mesh_filter_base.cpp
  src/sensor_model.cpp
  src/stereo_camera_model.cpp
  src/gl_renderer.cpp
  src/gl_mesh.cpp
  )

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${gl_LIBS} glut GLEW)

#set (NODELET_NAME moveit_mesh_filter_nodelet)
#add_library(${NODELET_NAME}
#  src/transform_provider.cpp
#  src/depth_self_filter_nodelet.cpp
#  )

#target_link_libraries(${NODELET_NAME} ${MOVEIT_LIB_NAME})

#install(TARGETS ${NODELET_NAME} LIBRARY DESTINATION lib)
install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
