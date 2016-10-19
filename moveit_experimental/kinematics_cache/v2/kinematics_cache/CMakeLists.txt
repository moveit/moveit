set(MOVEIT_LIB_NAME moveit_kinematics_cache)

add_library(${MOVEIT_LIB_NAME}
  src/kinematics_cache.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION 0.7.3)
# This line is needed to ensure that messages are done being built before this is built
add_dependencies(${MOVEIT_LIB_NAME} moveit_msgs_gencpp)

target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_model ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

