set(MOVEIT_LIB_NAME moveit_robot_state)

add_library(${MOVEIT_LIB_NAME}
  src/attached_body.cpp
  src/conversions.cpp
  src/robot_state.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_model moveit_kinematics_base moveit_transforms ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

# Unit tests
if(CATKIN_ENABLE_TESTING)
  find_package(moveit_resources REQUIRED)
  include_directories(${moveit_resources_INCLUDE_DIRS})

  catkin_add_gtest(test_robot_state test/test_kinematic.cpp)
  target_link_libraries(test_robot_state ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${MOVEIT_LIB_NAME})

  catkin_add_gtest(test_robot_state_complex test/test_kinematic_complex.cpp)
  target_link_libraries(test_robot_state_complex ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${MOVEIT_LIB_NAME})

  catkin_add_gtest(test_aabb test/test_aabb.cpp)
  target_link_libraries(test_aabb ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${MOVEIT_LIB_NAME})
endif()
