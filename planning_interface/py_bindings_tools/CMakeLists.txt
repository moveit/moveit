set(MOVEIT_LIB_NAME moveit_py_bindings_tools)

add_library(${MOVEIT_LIB_NAME} src/roscpp_initializer.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})

add_library(_moveit_roscpp_initializer src/roscpp_initializer_pymodule.cpp)
set_target_properties(_moveit_roscpp_initializer PROPERTIES OUTPUT_NAME _moveit_roscpp_initializer PREFIX "")
target_link_libraries(_moveit_roscpp_initializer ${catkin_LIBRARIES} ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} _moveit_roscpp_initializer LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
