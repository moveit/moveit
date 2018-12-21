set(MOVEIT_LIB_NAME moveit_trajectory_processing)

add_library(${MOVEIT_LIB_NAME}
  src/iterative_time_parameterization.cpp
  src/iterative_spline_parameterization.cpp
  src/trajectory_tools.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_state moveit_robot_trajectory ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  include_directories(${moveit_resources_INCLUDE_DIRS})
  catkin_add_gtest(test_time_parameterization test/test_time_parameterization.cpp)
  target_link_libraries(test_time_parameterization ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${MOVEIT_LIB_NAME})
endif()
