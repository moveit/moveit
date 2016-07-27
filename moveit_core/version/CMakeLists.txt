add_executable(moveit_version version.cpp)
install(TARGETS moveit_version RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
