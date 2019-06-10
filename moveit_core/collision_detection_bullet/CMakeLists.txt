set(MOVEIT_LIB_NAME moveit_collision_detection_bullet)

add_library(${MOVEIT_LIB_NAME}
  src/collision_common.cpp
  src/collision_robot_bt.cpp
  src/collision_world_bt.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} moveit_collision_detection ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${LIBFCL_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

add_library(collision_detector_bullet_plugin src/collision_detector_bt_plugin_loader.cpp)
set_target_properties(collision_detector_bullet_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(collision_detector_bullet_plugin ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME})


install(TARGETS ${MOVEIT_LIB_NAME} collision_detector_bullet_plugin
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(FILES ../collision_detector_bullet_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(moveit_resources REQUIRED)
  include_directories(${moveit_resources_INCLUDE_DIRS})
endif()
