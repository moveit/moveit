set(MOVEIT_LIB_NAME moveit_robot_self_filter)

add_library(${MOVEIT_LIB_NAME} src/self_mask.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_filter src/test_filter.cpp)
target_link_libraries(test_filter ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(self_filter src/self_filter.cpp)
#target_link_libraries(self_filter ${MOVEIT_LIB_NAME})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
