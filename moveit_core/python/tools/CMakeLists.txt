set(MOVEIT_LIB_NAME moveit_py_bindings_tools)

add_library(${MOVEIT_LIB_NAME}
  src/pybind_rosmsg_typecasters.cpp
  src/roscpp_initializer.cpp
)
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

install(
  TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

#catkin_lint cannot detect target declarations in functions, here in pybind11_add_module
#catkin_lint: ignore undefined_target

# moveit.binding_tools
pybind11_add_module(pymoveit_binding_tools pymoveit_binding_tools.cpp)
target_link_libraries(pymoveit_binding_tools PRIVATE ${MOVEIT_LIB_NAME})
set_target_properties(pymoveit_binding_tools PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(pymoveit_binding_tools PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})

install(TARGETS pymoveit_binding_tools
  LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
