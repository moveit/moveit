set(MOVEIT_LIB_NAME moveit_py_bindings_tools)

add_library(${MOVEIT_LIB_NAME} src/roscpp_initializer.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#catkin_lint cannot detect target declarations in functions, here in pybind11_add_module
#catkin_lint: ignore undefined_target

# moveit.tools
pybind11_add_module(pymoveit_tools
  src/wrap_python_roscpp_initializer.cpp
  )
target_link_libraries(pymoveit_tools PRIVATE ${MOVEIT_LIB_NAME})
set_target_properties(pymoveit_tools PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(pymoveit_tools PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})

install(TARGETS pymoveit_tools DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
