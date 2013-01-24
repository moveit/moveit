set(MOVEIT_LIB_NAME moveit_planning_scene_interface)

add_library(${MOVEIT_LIB_NAME}_python src/planning_scene_interface.cpp)
target_link_libraries(${MOVEIT_LIB_NAME}_python ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} moveit_py_bindings_tools)
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES OUTPUT_NAME _moveit_planning_scene_interface PREFIX "")

install(TARGETS ${MOVEIT_LIB_NAME}_python LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
