set(MOVEIT_LIB_NAME moveit_kinematics_cache)

add_library(${MOVEIT_LIB_NAME}
  src/kinematics_cache.cpp
)

target_link_libraries(${MOVEIT_LIB_NAME} moveit_kinematic_model ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

