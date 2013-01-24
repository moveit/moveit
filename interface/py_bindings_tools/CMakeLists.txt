set(MOVEIT_LIB_NAME moveit_py_bindings_tools)

add_library(${MOVEIT_LIB_NAME} src/roscpp_initializer.cpp src/py_conversions.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(_moveit_roscpp_initializer src/roscpp_initializer_pymodule.cpp)
set_target_properties(_moveit_roscpp_initializer PROPERTIES OUTPUT_NAME _moveit_roscpp_initializer PREFIX "")
target_link_libraries(_moveit_roscpp_initializer ${catkin_LIBRARIES} ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(TARGETS _moveit_roscpp_initializer DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
