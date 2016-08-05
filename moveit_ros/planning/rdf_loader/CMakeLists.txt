set(MOVEIT_LIB_NAME moveit_rdf_loader)

add_library(${MOVEIT_LIB_NAME} src/rdf_loader.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
