set(MOVEIT_LIB_NAME moveit_transforms)

add_library(${MOVEIT_LIB_NAME}
  src/transforms.cpp
)

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# Unit tests
catkin_add_gtest(test_transforms test/test_transforms.cpp)
target_link_libraries(test_transforms ${MOVEIT_LIB_NAME})
