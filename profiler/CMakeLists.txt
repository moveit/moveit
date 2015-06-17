set(MOVEIT_LIB_NAME moveit_profiler)

add_library(${MOVEIT_LIB_NAME}
  src/profiler.cpp)

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
