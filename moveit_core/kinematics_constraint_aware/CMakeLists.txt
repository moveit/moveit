set(MOVEIT_LIB_NAME moveit_kinematics_constraint_aware)

add_library(${MOVEIT_LIB_NAME} src/kinematics_constraint_aware.cpp)

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
