set(MOVEIT_LIB_NAME moveit_mesh_filter)

add_library(${MOVEIT_LIB_NAME}
  src/mesh_filter_base.cpp
  src/sensor_model.cpp
  src/stereo_camera_model.cpp
  src/gl_renderer.cpp
  src/gl_mesh.cpp
  )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${gl_LIBS} GLUT::GLUT ${GLEW_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  #catkin_lint: ignore_once env_var
  # Can only run this test if we have a display
  if (DEFINED ENV{DISPLAY} AND NOT $ENV{DISPLAY} STREQUAL "")
    catkin_add_gtest(mesh_filter_test test/mesh_filter_test.cpp)
    target_link_libraries(mesh_filter_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} moveit_mesh_filter)
    # TODO: remove if transition to gtest's new API TYPED_TEST_SUITE_P is finished
    target_compile_options(mesh_filter_test PRIVATE -Wno-deprecated-declarations)
  else()
    message("No display, will not configure tests for moveit_ros_perception/mesh_filter")
  endif()
endif()

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
