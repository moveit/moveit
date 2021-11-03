set(MOVEIT_LIB_NAME moveit_robot_interface)

#catkin_lint: ignore undefined_target
pybind_add_module(py${MOVEIT_LIB_NAME} src/wrap_python_robot_interface.cpp)
target_link_libraries(py${MOVEIT_LIB_NAME} PRIVATE ${catkin_LIBRARIES} ${Boost_LIBRARIES}
  moveit_common_planning_interface_objects)
set_target_properties(py${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(py${MOVEIT_LIB_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}")
if(WIN32)
  set_target_properties(py${MOVEIT_LIB_NAME} PROPERTIES SUFFIX .pyd)
endif(WIN32)

install(TARGETS py${MOVEIT_LIB_NAME}
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
