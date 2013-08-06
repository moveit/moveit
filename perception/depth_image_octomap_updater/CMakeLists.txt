set(MOVEIT_LIB_NAME moveit_depth_image_octomap_updater)

add_library(${MOVEIT_LIB_NAME}_core
  src/lazy_free_space_updater.cpp
  src/depth_image_octomap_updater.cpp
  )

set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")
target_link_libraries(${MOVEIT_LIB_NAME}_core moveit_mesh_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(${MOVEIT_LIB_NAME} src/updater_plugin.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  
install(TARGETS ${MOVEIT_LIB_NAME}_core ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
 