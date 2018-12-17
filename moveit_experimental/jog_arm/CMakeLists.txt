add_executable(jog_arm_server src/jog_arm/jog_arm_server.cpp)
add_dependencies(jog_arm_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(jog_arm_server ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(spacenav_to_twist src/jog_arm/teleop_examples/spacenav_to_twist.cpp)
add_dependencies(spacenav_to_twist ${catkin_EXPORTED_TARGETS})
target_link_libraries(spacenav_to_twist ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(xbox_to_twist src/jog_arm/teleop_examples/xbox_to_twist.cpp)
add_dependencies(xbox_to_twist ${catkin_EXPORTED_TARGETS})
target_link_libraries(xbox_to_twist ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

install(TARGETS jog_arm_server spacenav_to_twist xbox_to_twist
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
