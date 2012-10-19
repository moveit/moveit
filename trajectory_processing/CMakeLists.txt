set(MOVEIT_LIB_NAME moveit_trajectory_processing)

add_library(${MOVEIT_LIB_NAME}
  src/iterative_smoother.cpp
  src/trajectory_tools.cpp
)
target_link_libraries(${MOVEIT_LIB_NAME} moveit_planning_scene ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)
