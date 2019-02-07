set(MOVEIT_LIB_NAME moveit_kinematic_constraints)

add_compile_options(-Wno-cast-qual)  # suppress warning in tf/Vector3.h: https://github.com/ros/geometry/pull/167
add_library(${MOVEIT_LIB_NAME}
  src/kinematic_constraint.cpp
  src/utils.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_model moveit_kinematics_base moveit_robot_state moveit_collision_detection_fcl
  ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  include_directories(${moveit_resources_INCLUDE_DIRS})

  catkin_add_gtest(test_constraints test/test_constraints.cpp)
  target_link_libraries(test_constraints  ${MOVEIT_LIB_NAME})
endif()
