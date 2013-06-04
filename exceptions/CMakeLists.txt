set(MOVEIT_LIB_NAME moveit_exceptions)

add_library(${MOVEIT_LIB_NAME} src/exceptions.cpp)

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
