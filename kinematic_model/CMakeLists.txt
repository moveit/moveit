set(MOVEIT_LIB_NAME moveit_kinematic_model)

add_library(${MOVEIT_LIB_NAME}
  src/link_model.cpp
  src/joint_model.cpp
  src/fixed_joint_model.cpp
  src/revolute_joint_model.cpp
  src/prismatic_joint_model.cpp
  src/planar_joint_model.cpp
  src/floating_joint_model.cpp
  src/joint_model_group.cpp
  src/kinematic_model.cpp
  )

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
