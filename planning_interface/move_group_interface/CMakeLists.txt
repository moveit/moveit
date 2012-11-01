set(MOVEIT_LIB_NAME moveit_move_group_interface)

add_library(${MOVEIT_LIB_NAME} src/move_group.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(${MOVEIT_LIB_NAME}_python src/wrap_python_move_group.cpp)
target_link_libraries(${MOVEIT_LIB_NAME}_python ${MOVEIT_LIB_NAME} ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES OUTPUT_NAME _move_group PREFIX "")

install(TARGETS ${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_python LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
